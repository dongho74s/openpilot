import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

import { TraverseLiveLink, decodeVizCan, sameOriginSocketBase } from "../src/features/drive/contents/traverse_live/link.js";
import { LiveState } from "../src/features/drive/contents/traverse_live/state.js";
import { LaneFilter } from "../src/features/drive/contents/traverse_live/lanes.js";
import { Tracker } from "../src/features/drive/contents/traverse_live/tracker.js";
import { hudText } from "../src/features/drive/contents/traverse_live/hud.js";
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
