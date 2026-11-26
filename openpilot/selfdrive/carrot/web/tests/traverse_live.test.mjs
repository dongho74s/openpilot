import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import { TraverseLiveLink, decodeVizCan, sameOriginSocketBase } from "../src/features/drive/contents/traverse_live/link.js";
import { LiveState } from "../src/features/drive/contents/traverse_live/state.js";
import { LaneFilter } from "../src/features/drive/contents/traverse_live/lanes.js";
import { Tracker } from "../src/features/drive/contents/traverse_live/tracker.js";
import { externalCarrotMan, formatDistance, hudText } from "../src/features/drive/contents/traverse_live/hud.js";
import {
  TRAVERSE_LIVE_BUNDLE_ID,
  TRAVERSE_LIVE_CONTENT_ID,
  requestTraverseLiveInPlaceOfVision,
  resolveTraverseLiveBundleUrl,
} from "../src/features/drive/contents/traverse_live/content.js";
import { replaceLayoutContents } from "../src/features/drive/core/workspace_runtime.js";

const catalog = JSON.parse(readFileSync(new URL("../src/features/drive/core/content_catalog.json", import.meta.url), "utf8"));

function vizCanBatch(records) {
  const out = new Uint8Array(5 + records.length * 14);
  const dv = new DataView(out.buffer);
  out[0] = 0x56; out[1] = 0x43; out[2] = 2; dv.setUint16(3, records.length, true);
  records.forEach(({ bus, addr, ageMs, data }, i) => {
    const o = 5 + i * 14;
    out[o] = bus; dv.setUint16(o + 1, addr, true); dv.setUint16(o + 3, ageMs, true); out[o + 5] = data.length;
    out.set(data, o + 6);
  });
  return out;
}

test("vizCan v2 batches decode into bus/addr/age/data frames", () => {
  const frames = decodeVizCan(vizCanBatch([
    { bus: 1, addr: 0x421, ageMs: 40, data: [1, 2, 3, 4, 5, 6, 7, 8] },
    { bus: 2, addr: 0x510, ageMs: 0, data: [9, 9, 0x0C] },
  ]));
  assert.equal(frames.length, 2);
  assert.deepEqual([frames[0].bus, frames[0].addr, frames[0].age], [1, 0x421, 0.04]);
  assert.deepEqual([...frames[0].d], [1, 2, 3, 4, 5, 6, 7, 8]);
  assert.deepEqual([...frames[1].d], [9, 9, 0x0C]);
  assert.equal(decodeVizCan(new Uint8Array([1, 2, 3])), null);
});

test("the socket goes to the comma that served the page", () => {
  assert.equal(sameOriginSocketBase({ protocol: "http:", host: "100.78.55.7:7000" }), "ws://100.78.55.7:7000");
  assert.equal(sameOriginSocketBase({ protocol: "https:", host: "comma.local" }), "wss://comma.local");
});

class FakeSocket {
  static all = [];
  constructor(url) { this.url = url; FakeSocket.all.push(this); }
  close() { this.onclose?.(); }
}

test("one /ws/viz socket; falls back to raw_multiplex + viz_can when the comma has no /ws/viz", () => {
  FakeSocket.all = [];
  const seen = [];
  const decoder = { decodeHudEvent: (svc) => ({ svc, hud: true }), decodeOverlayEvent: (svc) => ({ svc, hud: false }) };
  const link = new TraverseLiveLink({ raw: (svc, d) => seen.push([svc, d.hud]), can: (frames) => seen.push(["can", frames.length]) },
    { base: "ws://c:7000", WebSocket: FakeSocket, decoder: () => decoder });
  link.start();
  assert.deepEqual(FakeSocket.all.map((s) => s.url), ["ws://c:7000/ws/viz"]);
  const viz = FakeSocket.all[0];
  viz.onopen();
  assert.equal(link.isOpen(), true);
  const frame = (name, payload) => {
    const bytes = new TextEncoder().encode(name);
    const out = new Uint8Array(1 + bytes.length + payload.length);
    out[0] = bytes.length; out.set(bytes, 1); out.set(payload, 1 + bytes.length);
    return out.buffer;
  };
  viz.onmessage({ data: frame("carState", new Uint8Array([0])) });
  viz.onmessage({ data: frame("modelV2", new Uint8Array([0])) });
  viz.onmessage({ data: frame("vizCan", vizCanBatch([{ bus: 1, addr: 0x420, ageMs: 0, data: [0] }])) });
  assert.deepEqual(seen, [["carState", true], ["modelV2", false], ["can", 1]]);
  link.stop();

  FakeSocket.all = [];
  const legacy = new TraverseLiveLink({}, { base: "ws://c:7000", WebSocket: FakeSocket, decoder: () => decoder });
  legacy.start();
  FakeSocket.all[0].onclose();   // closed before it ever opened: old comma build
  assert.deepEqual(FakeSocket.all.slice(1).map((s) => s.url.split("?")[0]), ["ws://c:7000/ws/raw_multiplex", "ws://c:7000/ws/viz_can"]);
  assert.equal(legacy.legacy, true);
  legacy.stop();
});

test("live state follows carState, CAN frames and the lead-car lamps like the app", () => {
  const st = new LiveState();
  st.onCarState({ vEgo: 20, leftBlinker: true, gearShifter: "drive", cruiseState: { enabled: true }, vCruiseCluster: 90,
    stockLaneMarkings: { supported: true, left: 1, right: 0 } }, 10);
  assert.equal(st.cs.gear, "D");
  assert.equal(st.cs.cspdKph, 90);
  assert.deepEqual(st.cs.slm, [1, 0]);
  st.onCanFrames([{ bus: 1, addr: 0x427, age: 0, d: new Uint8Array([0, 0, 0, 1, 0x40, 0, 0, 0]) }], 10);
  assert.ok(st.frame(1, 0x427, 10));
  assert.equal(st.frame(1, 0x427, 11), null);
});

test("buildFrame turns a model into road, four lanes and the path (view.js)", async () => {
  // view.js imports scene.js (three.js, from node_modules like the AR tests); only the pure frame build runs here.
  const { buildFrame } = await import("../src/features/drive/contents/traverse_live/view.js");
  const st = new LiveState();
  const xs = Array.from({ length: 33 }, (_, i) => i * 3);
  const line = (y) => ({ x: xs, y: xs.map(() => y) });
  st.onCarState({ vEgo: 15, rightBlinker: true, gearShifter: "drive" }, 1);
  st.onCarControl({ latActive: true, longActive: true });
  st.onModel({ laneLines: [line(-5.4), line(-1.8), line(1.8), line(5.4)], laneLineProbs: [0.6, 0.9, 0.9, 0.6],
    roadEdges: [line(-7), line(7)], position: line(0) }, 1);
  const { fr } = buildFrame(st, new LaneFilter(), new Tracker(), 1);
  assert.equal(fr.lines.length, 4);
  assert.ok(fr.road);
  assert.equal(fr.path.on, true);
  assert.equal(fr.edges.length, 2);
});

test("HUD labels follow the page language", () => {
  assert.equal(hudText("ko").brake, "브레이크");
  assert.equal(hudText("en-US").manual, "Manual");
  assert.equal(hudText("zh").hazard, "双闪");
  assert.equal(hudText("fr").brake, "Brake");
});

test("Traverse Live is a live-only drive content in both areas", () => {
  const entry = catalog.contents.find((c) => c.id === TRAVERSE_LIVE_CONTENT_ID);
  assert.deepEqual(entry, {
    id: "traverse_live",
    labelKey: "web_drive_layout_content_traverse_live",
    supportedSlots: ["primary", "secondary"],
    supportedSources: ["live"],
    singleton: true,
  });
  assert.equal(resolveTraverseLiveBundleUrl({ CarrotAssetUrl: { resolve: (id) => `/x/${id}?v=1` } }), `/x/${TRAVERSE_LIVE_BUNDLE_ID}?v=1`);
  assert.equal(resolveTraverseLiveBundleUrl({}), "/js/generated/traverse-live.js");
});

test("starting from the vision start screen takes Carrot Vision's slot, never the saved layout", () => {
  const known = () => true;
  assert.deepEqual(replaceLayoutContents({ area1Content: "vision", area2Content: "navigation" }, { from: "vision", to: "traverse_live" }, known),
    { primaryId: "traverse_live", secondaryId: "navigation" });
  // Already in area 2: the two swap instead of showing it twice.
  assert.deepEqual(replaceLayoutContents({ area1Content: "vision", area2Content: "traverse_live" }, { from: "vision", to: "traverse_live" }, known),
    { primaryId: "traverse_live", secondaryId: "vision" });
  assert.deepEqual(replaceLayoutContents({ area1Content: "navigation", area2Content: "vision" }, { from: "vision", to: "traverse_live" }, known),
    { primaryId: "navigation", secondaryId: "traverse_live" });
  assert.deepEqual(replaceLayoutContents({ area1Content: "vision", area2Content: "navigation" }, null, known),
    { primaryId: "vision", secondaryId: "navigation" });
  assert.deepEqual(replaceLayoutContents({ area1Content: "vision", area2Content: "navigation" }, { from: "vision", to: "nope" }, () => false),
    { primaryId: "vision", secondaryId: "navigation" });

  const calls = [];
  const target = { DriveWorkspaceRuntime: { replaceContent: (from, to) => { calls.push([from, to]); return true; }, slotForContent: () => null } };
  assert.equal(requestTraverseLiveInPlaceOfVision(target), true);
  assert.deepEqual(calls, [["vision", "traverse_live"]]);
  assert.equal(requestTraverseLiveInPlaceOfVision({}), false);
});

test("3D view camera: the default view is the app's, zoom and look-around stay in range", async () => {
  const { RIG_DEFAULT, RIG_LIMITS, clampRig, createCameraRig, rigEye, paintFor } = await import("../src/features/drive/contents/traverse_live/scene.js");
  const eye = rigEye(createCameraRig());
  assert.ok(Math.abs(eye[0] + 12.5) < 1e-9 && Math.abs(eye[1]) < 1e-9 && Math.abs(eye[2] - 6.4) < 1e-9);
  assert.equal(clampRig("zoom", 10), RIG_LIMITS.zoom[1]);
  assert.equal(clampRig("zoom", 0.1), RIG_LIMITS.zoom[0]);
  assert.equal(clampRig("yaw", -200), RIG_LIMITS.yaw[0]);
  // Zoomed out the camera is further back and higher than the default.
  const far = rigEye({ ...RIG_DEFAULT, zoom: 2 });
  assert.ok(far[0] < eye[0] && far[2] > eye[2]);
  // A car keeps its paint; the palette has more than one colour.
  assert.equal(paintFor(7), paintFor(7));
  const colours = new Set(Array.from({ length: 40 }, (_, i) => paintFor(i).getHexString()));
  assert.ok(colours.size >= 4);
});

test("3D view picks a vehicle kind from the camera class and width, stable per track", async () => {
  const { vehicleModel } = await import("../src/features/drive/contents/traverse_live/view.js");
  assert.equal(vehicleModel({ type: 4, w: 0.5, uid: 1 }), "ped");
  assert.equal(vehicleModel({ type: 3, w: 0.7, uid: 1 }), "moto");
  assert.equal(vehicleModel({ type: 2, w: 2.5, uid: 1 }), "truck");
  assert.equal(vehicleModel({ type: 2, w: 2.0, uid: 1 }), "pickup");
  assert.ok(["sedan", "hatch", "suv", "suvl"].includes(vehicleModel({ type: 1, w: 1.75, uid: 3 })));
  assert.ok(["suv", "suvl", "van"].includes(vehicleModel({ type: 1, w: 2.0, uid: 3 })));
  assert.equal(vehicleModel({ type: 1, w: 1.75, uid: 9 }), vehicleModel({ type: 1, w: 1.75, uid: 9 }));
  const models = JSON.parse(readFileSync(new URL("../assets/traverse_live/models_kenney.json", import.meta.url), "utf8"));
  for (const key of ["sedan", "suv", "suvl", "hatch", "van", "pickup", "truck"]) assert.ok(models[key]?.groups?.body, key);
});

test("speed limit / next turn / signal are shown only from carrotMan the web already receives", () => {
  const now = 1_000_000;
  assert.equal(externalCarrotMan({}, now), null);
  const target = {
    CarrotHudState: { carrotMan: { nRoadLimitSpeed: 60, xSpdLimit: 50, xSpdDist: 320, xDistToTurn: 450, szTBTMainText: "청라커낼로", trafficState: 1 } },
    CarrotStateUpdatedAt: { carrotMan: now - 500 },
  };
  assert.deepEqual(externalCarrotMan(target, now),
    { roadLimit: 60, camLimit: 50, camDist: 320, turnDist: 450, turnText: "청라커낼로", traffic: "red" });
  target.CarrotStateUpdatedAt.carrotMan = now - 5000;   // stale: the 3D view never opens a stream for it
  assert.equal(externalCarrotMan(target, now), null);
  assert.equal(formatDistance(447), "450 m");
  assert.equal(formatDistance(1530), "1.5 km");
  assert.equal(formatDistance(0), "");
});

test("the 3D view name is what the drive tab shows", () => {
  for (const [lang, label] of [["ko", "3D 뷰"], ["en", "3D View"], ["zh", "3D 视图"]]) {
    const source = readFileSync(new URL(`../js/translations/${lang}.js`, import.meta.url), "utf8");
    assert.ok(source.includes(`web_drive_layout_content_traverse_live: "${label}"`), lang);
    assert.ok(!source.includes("Traverse Live"), lang);
  }
});
