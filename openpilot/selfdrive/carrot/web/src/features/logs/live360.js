"use strict";

// 360 live view for the Log section (Logs > Live).
//
// Shows the wide-angle road camera and the indoor driver camera side by
// side over a single WebRTC session (cameras: ["road", "driver"]). The
// server sends one video track per camera; the track order follows the
// cameras list, so transceiver mid "0" is road and "1" is driver.
//
// The session starts when the Live tab becomes active and stops when the
// user leaves it, so the camera pipeline (and battery) is only used while
// someone is actually watching. While driving it just works; while parked
// it needs the "주차 중 360 라이브" toggle (CarrotVisionParkedEnabled),
// which is also offered right here in the panel.

const LIVE360_CAMERAS = Object.freeze(["road", "driver"]);
const LIVE360_BUSY_CODE = "carrot_vision_busy";
const LIVE360_ICE_TIMEOUT_MS = 1500;
const LIVE360_FETCH_TIMEOUT_MS = 10000;
const LIVE360_PARKED_STARTUP_WAIT_MS = 4000;

const live360Els = {
  roadVideo: null,
  driverVideo: null,
  status: null,
  toggle: null,
  retry: null,
};

let live360Pc = null;
let live360Generation = 0;
let live360Started = false;
let live360Starting = false;
let live360TrackCount = 0;

function live360DeviceId() {
  return String(
    window.CarrotStreamIdentity?.deviceId
    || window.CarrotStreamIdentity?.clientId
    || `carrot-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`,
  );
}

function live360TabId() {
  return String(window.CarrotStreamIdentity?.tabId || "");
}

function live360AttemptId() {
  try {
    const generated = window.CarrotStreamIdentity?.createAttemptId?.();
    if (generated) return String(generated).slice(0, 128);
  } catch (_) {}
  return `live360-attempt-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`.slice(0, 128);
}

function live360SetStatus(text) {
  if (live360Els.status) live360Els.status.textContent = text || "";
}

function live360ShowRetry(show) {
  if (live360Els.retry) live360Els.retry.hidden = !show;
}

async function live360FetchJson(url, options = {}) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), LIVE360_FETCH_TIMEOUT_MS);
  try {
    const response = await fetch(url, { ...options, signal: controller.signal });
    const text = await response.text();
    let payload = null;
    try { payload = text ? JSON.parse(text) : null; } catch (_) {}
    return { ok: response.ok, status: response.status, payload, text };
  } finally {
    clearTimeout(timer);
  }
}

async function live360ReadParams() {
  const names = ["CarrotVisionEnabled", "CarrotVisionParkedEnabled", "IsOnroad", "IsOffroad"];
  const result = await live360FetchJson(`/api/params_bulk?names=${encodeURIComponent(names.join(","))}`, { cache: "no-store" });
  if (!result.ok) throw new Error("params_bulk HTTP " + result.status);
  const values = result.payload?.values || {};
  const truthy = (v) => v === true || v === 1 || v === "1" || v === "true";
  return {
    visionEnabled: truthy(values.CarrotVisionEnabled),
    parkedEnabled: truthy(values.CarrotVisionParkedEnabled),
    isOnroad: truthy(values.IsOnroad),
    isOffroad: truthy(values.IsOffroad),
  };
}

async function live360SetParkedEnabled(enabled) {
  const result = await live360FetchJson("/api/param_set", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name: "CarrotVisionParkedEnabled", value: enabled ? 1 : 0, source: "web_ui" }),
  });
  if (!result.ok) throw new Error(result.payload?.error || ("param_set HTTP " + result.status));
  return result.payload?.value;
}

function live360ClearVideos() {
  for (const video of [live360Els.roadVideo, live360Els.driverVideo]) {
    if (!video) continue;
    try {
      const stream = video.srcObject;
      video.srcObject = null;
      if (stream && typeof stream.getTracks === "function") {
        for (const track of stream.getTracks()) { try { track.stop(); } catch (_) {} }
      }
    } catch (_) {}
  }
}

function live360ClosePeer() {
  const pc = live360Pc;
  live360Pc = null;
  if (!pc) return;
  try { pc.ontrack = null; } catch (_) {}
  try { pc.onconnectionstatechange = null; } catch (_) {}
  try { pc.oniceconnectionstatechange = null; } catch (_) {}
  try { pc.close(); } catch (_) {}
}

function live360AttachTrack(track, mid) {
  const isDriver = mid === "1" || (mid !== "0" && live360TrackCount >= 1);
  const video = isDriver ? live360Els.driverVideo : live360Els.roadVideo;
  live360TrackCount += 1;
  if (!video) return;
  let stream = video.srcObject;
  if (!stream || typeof stream.addTrack !== "function") {
    stream = new MediaStream([track]);
    video.srcObject = stream;
  } else if (!stream.getTracks().includes(track)) {
    stream.addTrack(track);
  }
  video.play().catch(() => {});
  live360SetStatus("라이브 연결됨");
  live360ShowRetry(false);
}

function live360WaitIceComplete(pc, timeoutMs) {
  if (pc.iceGatheringState === "complete") return Promise.resolve(true);
  return new Promise((resolve) => {
    let done = false;
    const finish = (value) => { if (!done) { done = true; resolve(value); } };
    const timer = setTimeout(() => finish(false), timeoutMs);
    const check = () => {
      if (pc.iceGatheringState === "complete") { clearTimeout(timer); finish(true); }
    };
    pc.addEventListener("icegatheringstatechange", check);
    setTimeout(() => { try { pc.removeEventListener("icegatheringstatechange", check); } catch (_) {} }, timeoutMs + 100);
  });
}

async function live360Connect(generation) {
  const pc = new RTCPeerConnection();
  live360Pc = pc;
  live360TrackCount = 0;

  pc.addTransceiver("video", { direction: "recvonly" });
  pc.addTransceiver("video", { direction: "recvonly" });

  pc.ontrack = (ev) => {
    if (generation !== live360Generation || live360Pc !== pc) return;
    if (!ev.track || ev.track.kind !== "video") return;
    const mid = ev.transceiver?.mid ?? "";
    live360AttachTrack(ev.track, String(mid));
  };

  pc.onconnectionstatechange = () => {
    if (generation !== live360Generation || live360Pc !== pc) return;
    if (pc.connectionState === "failed" || pc.connectionState === "closed") {
      live360SetStatus("연결이 끊겼어요. 다시 연결을 눌러주세요.");
      live360ShowRetry(true);
    }
  };

  const offer = await pc.createOffer();
  await pc.setLocalDescription(offer);
  await live360WaitIceComplete(pc, LIVE360_ICE_TIMEOUT_MS);
  if (generation !== live360Generation || live360Pc !== pc) return;

  const body = {
    sdp: pc.localDescription.sdp,
    cameras: [...LIVE360_CAMERAS],
    bridge_services_in: [],
    bridge_services_out: [],
    client_id: live360DeviceId(),
    device_id: live360DeviceId(),
    tab_id: live360TabId(),
    attempt_id: live360AttemptId(),
    takeover: false,
    carrot_state: false,
  };

  const response = await live360FetchJson("/stream", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (generation !== live360Generation || live360Pc !== pc) return;

  if (!response.ok) {
    if (response.status === 409 && response.payload?.code === LIVE360_BUSY_CODE) {
      const error = new Error("busy");
      error.code = LIVE360_BUSY_CODE;
      throw error;
    }
    throw new Error("stream HTTP " + response.status + " " + (response.text || ""));
  }
  const answer = response.payload;
  if (!answer || !answer.sdp) throw new Error("bad answer");
  await pc.setRemoteDescription({ type: answer.type || "answer", sdp: answer.sdp });
}

export async function startLive360() {
  if (live360Started || live360Starting) return;
  if (!live360Els.roadVideo || !live360Els.driverVideo) {
    if (!initLive360()) return;
  }
  live360Starting = true;
  const generation = ++live360Generation;
  live360ShowRetry(false);
  try {
    const params = await live360ReadParams();
    if (generation !== live360Generation) return;
    if (live360Els.toggle) live360Els.toggle.checked = !!params.parkedEnabled;

    if (!params.visionEnabled) {
      live360SetStatus("Carrot Vision이 꺼져 있어요. 설정에서 Carrot Vision을 켜주세요.");
      live360ShowRetry(false);
      return;
    }
    const parked = params.isOffroad || !params.isOnroad;
    if (parked && !params.parkedEnabled) {
      live360SetStatus("주차 중에는 아래 토글을 켜야 라이브가 보입니다.");
      live360ShowRetry(false);
      return;
    }

    live360SetStatus("연결 중…");
    await live360Connect(generation);
    if (generation !== live360Generation) return;
    live360Started = true;
    live360SetStatus("연결됨, 영상 기다리는 중…");
  } catch (error) {
    if (generation !== live360Generation) return;
    live360ClosePeer();
    if (error?.code === LIVE360_BUSY_CODE) {
      live360SetStatus("다른 화면에서 라이브를 보고 있어요. 그 화면을 닫고 다시 연결을 눌러주세요.");
    } else {
      live360SetStatus("연결에 실패했어요. 다시 연결을 눌러주세요.");
    }
    live360ShowRetry(true);
  } finally {
    if (generation === live360Generation) live360Starting = false;
  }
}

export function stopLive360() {
  live360Generation += 1;
  live360Started = false;
  live360Starting = false;
  live360ClosePeer();
  live360ClearVideos();
  live360ShowRetry(false);
}

export function isLive360Active() {
  return live360Started;
}

async function live360OnToggleChange(event) {
  const enabled = !!event?.target?.checked;
  if (live360Els.toggle) live360Els.toggle.disabled = true;
  try {
    await live360SetParkedEnabled(enabled);
    if (enabled) {
      // The manager needs a moment to start camerad / encoder / webrtcd.
      live360SetStatus("카메라를 켜는 중… 잠시만 기다려주세요.");
      await new Promise((resolve) => setTimeout(resolve, LIVE360_PARKED_STARTUP_WAIT_MS));
      stopLive360();
      startLive360().catch(() => {});
    } else {
      live360SetStatus("주차 중 라이브를 껐어요.");
      stopLive360();
    }
  } catch (_) {
    if (live360Els.toggle) live360Els.toggle.checked = !enabled;
    live360SetStatus("토글 저장에 실패했어요. 다시 시도해주세요.");
  } finally {
    if (live360Els.toggle) live360Els.toggle.disabled = false;
  }
}

export function initLive360() {
  live360Els.roadVideo = document.getElementById("live360RoadVideo");
  live360Els.driverVideo = document.getElementById("live360DriverVideo");
  live360Els.status = document.getElementById("live360Status");
  live360Els.toggle = document.getElementById("live360ParkedToggle");
  live360Els.retry = document.getElementById("live360Retry");
  if (!live360Els.roadVideo || !live360Els.driverVideo) return false;
  live360Els.toggle?.addEventListener("change", live360OnToggleChange);
  live360Els.retry?.addEventListener("click", () => {
    stopLive360();
    startLive360().catch(() => {});
  });
  return true;
}
