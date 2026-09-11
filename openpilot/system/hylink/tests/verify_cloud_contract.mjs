// Offline contract check against the actual Wayon Worker source (D1/KV are in-memory fakes).
// Usage: python ...produce a real cereal telemetry JSON... | node verify_cloud_contract.mjs /path/to/worker.js
import assert from "node:assert/strict";
import { pathToFileURL } from "node:url";

// Cloudflare accepts copied request streams; Node's Undici additionally requires duplex.
const NativeRequest = globalThis.Request;
globalThis.Request = class extends NativeRequest {
  constructor(input, init = {}) {
    super(input, init.body instanceof ReadableStream ? { ...init, duplex: "half" } : init);
  }
};
globalThis.fetch = async () => { throw new Error("External network forbidden in contract test"); };
const { default: worker } = await import(pathToFileURL(process.argv[2]).href);
let input = "";
for await (const chunk of process.stdin) input += chunk;
const payload = JSON.parse(input);
const deviceId = payload.deviceId;
const key = "wayon_" + "t".repeat(48);
let stored = null;
const env = {
  SNAPSHOTS: {},
  DB: {
    prepare(query) {
      let values = [];
      const statement = {
        bind(...args) { values = args; return statement; },
        async first() {
          if (/FROM wayon_devices/.test(query)) return { device_id: deviceId };
          if (/FROM latest_state/.test(query)) return stored;
          return null;
        },
        async all() { return { results: [] }; },
        async run() {
          if (/INSERT INTO latest_state/.test(query)) {
            const columns = ["device_id", "updated_at", "onroad", "ignition", "enabled", "voltage_v",
              "current_ma", "power_w", "device_power_w", "thermal_status", "fan_percent",
              "screen_brightness_percent", "latitude", "longitude", "speed_mps", "bearing_deg",
              "gps_accuracy_m", "raw_json"];
            assert.equal(values.length, columns.length);
            stored = Object.fromEntries(columns.map((name, index) => [name, values[index]]));
          }
          return { success: true };
        },
      };
      return statement;
    },
  },
};
const headers = { authorization: "Bearer " + key, "content-type": "application/json" };
const upload = await worker.fetch(new Request("https://test.invalid/api/telemetry", {
  method: "POST", headers, body: JSON.stringify(payload),
}), env, {});
assert.equal(upload.status, 200);
assert.equal(stored.device_id, deviceId);
assert.equal(stored.speed_mps, payload.vehicleSpeedMps);
assert.equal(stored.voltage_v, payload.voltageV);
assert.equal(stored.current_ma, payload.currentMa);
const feed = await worker.fetch(new Request("https://test.invalid/api/state", { headers }), env, {});
assert.equal(feed.status, 200);
const json = await feed.json();
const raw = JSON.parse(json.state.raw_json);
assert.equal(raw.schemaVersion, "wayon-telemetry-v3");
assert.equal(raw.vehicle.can.valid, payload.vehicle.can.valid);
assert.equal(raw.gps.latitude, payload.gps.latitude);
assert.equal(raw.openpilot.alert.text1, payload.openpilot.alert.text1);
if (payload.hylink) assert.deepEqual(raw.hylink, payload.hylink);
assert.equal(raw.onroad, payload.onroad);
console.log("PASS: current cereal -> new uploader -> actual Wayon Worker -> Hylink /api/state contract (no network)");
