/* Drive tab content "traverse_live": the Traverse Live phone app's driving scene in a drive slot.
 *
 * This module is part of the always-loaded drive bundle and stays small: the
 * three.js scene, HUD and socket live in js/generated/traverse-live.js and are
 * only downloaded when someone starts the view, together with the carrot web's
 * raw capnp decoder and the vehicle models. The comma only streams /ws/viz while
 * the view runs; the viewing device does all the drawing. */

export const TRAVERSE_LIVE_CONTENT_ID = "traverse_live";
export const TRAVERSE_LIVE_SUPPORTED_SLOTS = Object.freeze(["primary", "secondary"]);
export const TRAVERSE_LIVE_SUPPORTED_SOURCES = Object.freeze(["live"]);
export const TRAVERSE_LIVE_BUNDLE_ID = "drive.traverse-live";
export const TRAVERSE_LIVE_MODELS_URL = "/assets/traverse_live/models_kenney.json";
// Workers pin older copies with their own ?v=; this key carries carState.stockLaneMarkings.
export const TRAVERSE_LIVE_DECODER_URL = "/js/realtime/raw_capnp.js?v=traverse-live-1";
export const TRAVERSE_LIVE_START_BUTTON_ID = "btnStartTraverseLive";

const STYLE_ID = "carrot-traverse-live-content-style";
const CSS = `
.ctl-content{position:relative;width:100%;height:100%;min-height:0;overflow:hidden;background:#06080a;color:#eef1f4;
  --ctl-tr-gap:64px}
.ctl-content[hidden],.ctl-content [hidden]{display:none!important}
.ctl-host{position:absolute;inset:0}
.ctl-panel{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:12px;
  padding:16px;text-align:center;background:linear-gradient(#06080a,#16191c)}
.ctl-panel__title{font-size:1.25rem;font-weight:700;letter-spacing:.01em}
.ctl-panel__desc{max-width:420px;font-size:.92rem;line-height:1.5;color:#8a929a}
.ctl-panel__message{min-height:1.4em;font-size:.88rem;color:#d8a24a}
.ctl-panel .btn{font-size:1.1rem;padding:14px 30px;border-radius:99px}
.ctl-close{position:absolute;top:10px;right:10px;z-index:3;padding:6px 14px;border-radius:99px;border:1px solid rgba(255,255,255,.18);
  background:rgba(20,23,26,.78);color:#eef1f4;font:inherit;font-size:.85rem;cursor:pointer}
.ctl-close:focus-visible{outline:2px solid #3088ff;outline-offset:2px}
.vision-start-overlay .vision-start-overlay__alt{border:1px solid rgba(255,255,255,.24);background:rgba(16,19,22,.72);color:#eef1f4;
  font-weight:700;box-shadow:none}
.vision-start-overlay .vision-start-overlay__alt:hover{background:rgba(40,45,50,.86)}
.vision-start-overlay .vision-start-overlay__alt:focus-visible{outline:2px solid #eef1f4;outline-offset:2px}
`;

const contentSingletons = new WeakMap();
const scriptLoads = new Map();
let modelsLoad = null;
let pendingAutoStart = false;

function uiText(target, key, fallback) {
  try {
    const value = target.getUIText?.(key, fallback);
    if (typeof value === "string" && value) return value;
  } catch (_) {}
  const lang = String(target.document?.documentElement?.lang || "en").toLowerCase().split(/[-_]/)[0];
  const table = target.CarrotTranslations?.strings?.[lang] || {};
  return typeof table[key] === "string" && table[key] ? table[key] : fallback;
}

function injectStyle(doc) {
  if (!doc || doc.getElementById(STYLE_ID)) return;
  const style = doc.createElement("style");
  style.id = STYLE_ID;
  style.textContent = CSS;
  (doc.head || doc.documentElement).appendChild(style);
}

function loadScript(doc, src) {
  if (scriptLoads.has(src)) return scriptLoads.get(src);
  const promise = new Promise((resolve, reject) => {
    const script = doc.createElement("script");
    script.src = src;
    script.async = true;
    script.onload = () => resolve(true);
    script.onerror = () => {
      scriptLoads.delete(src);
      script.remove();
      reject(new Error(`load failed: ${src}`));
    };
    (doc.head || doc.documentElement).appendChild(script);
  });
  scriptLoads.set(src, promise);
  return promise;
}

export function resolveTraverseLiveBundleUrl(target = globalThis) {
  try {
    const url = target.CarrotAssetUrl?.resolve?.(TRAVERSE_LIVE_BUNDLE_ID);
    if (url) return String(url);
  } catch (_) {}
  return "/js/generated/traverse-live.js";
}

async function loadModels(target) {
  if (!modelsLoad) {
    modelsLoad = target.fetch(TRAVERSE_LIVE_MODELS_URL)
      .then((response) => {
        if (!response.ok) throw new Error(`models ${response.status}`);
        return response.json();
      })
      .catch((error) => {
        modelsLoad = null;
        throw error;
      });
  }
  return modelsLoad;
}

export async function loadTraverseLiveRuntime(target = globalThis) {
  const doc = target.document;
  const loads = [];
  if (!target.CarrotRawCapnp) loads.push(loadScript(doc, TRAVERSE_LIVE_DECODER_URL));
  if (!target.CarrotTraverseLive) loads.push(loadScript(doc, resolveTraverseLiveBundleUrl(target)));
  const [models] = await Promise.all([loadModels(target), ...loads]);
  if (!target.CarrotTraverseLive?.createView || !target.CarrotRawCapnp) throw new Error("Traverse Live runtime is unavailable");
  return { runtime: target.CarrotTraverseLive, models };
}

// The workspace shows Traverse Live where Carrot Vision was, and back again.
function workspaceRuntime(target) {
  return target.DriveWorkspaceRuntime || null;
}

export function requestTraverseLiveInPlaceOfVision(target = globalThis) {
  const runtime = workspaceRuntime(target);
  const existing = contentSingletons.get(target);
  if (existing && runtime?.slotForContent?.(TRAVERSE_LIVE_CONTENT_ID)) {
    existing.requestStart();
    return true;
  }
  pendingAutoStart = true;
  if (!runtime?.replaceContent?.("vision", TRAVERSE_LIVE_CONTENT_ID)) {
    pendingAutoStart = false;
    return false;
  }
  return true;
}

export function createTraverseLiveContent(options = {}) {
  const target = options.target || globalThis;
  const driveContentApi = options.driveContentApi || target.DriveContent;
  const doc = target.document;
  if (typeof driveContentApi?.create !== "function" || !doc) return null;
  injectStyle(doc);

  const root = doc.createElement("div");
  root.className = "ctl-content";
  root.dataset.driveContent = TRAVERSE_LIVE_CONTENT_ID;
  const host = doc.createElement("div");
  host.className = "ctl-host";
  const panel = doc.createElement("div");
  panel.className = "ctl-panel";
  const title = doc.createElement("div");
  title.className = "ctl-panel__title";
  title.textContent = "Traverse Live";
  const desc = doc.createElement("div");
  desc.className = "ctl-panel__desc";
  const startButton = doc.createElement("button");
  startButton.type = "button";
  startButton.className = "btn btn--filled";
  const message = doc.createElement("div");
  message.className = "ctl-panel__message";
  message.setAttribute("aria-live", "polite");
  panel.append(title, desc, startButton, message);
  const closeButton = doc.createElement("button");
  closeButton.type = "button";
  closeButton.className = "ctl-close";
  closeButton.hidden = true;
  root.append(host, panel, closeButton);

  let active = false;
  let loading = false;
  let error = "";
  let view = null;
  let startToken = 0;
  let mountedHost = null;

  function render() {
    desc.textContent = uiText(target, "traverse_live_desc",
      "The Traverse Live driving scene. This device draws it; the comma only sends data.");
    startButton.textContent = loading
      ? uiText(target, "traverse_live_loading", "Loading…")
      : `▶ ${uiText(target, "start_traverse_live", "Start Traverse Live")}`;
    startButton.disabled = loading;
    message.textContent = error;
    closeButton.textContent = uiText(target, "traverse_live_close", "Close");
    panel.hidden = Boolean(view);
    closeButton.hidden = !view;
  }

  async function start() {
    if (view || loading) return false;
    const token = ++startToken;
    loading = true;
    error = "";
    render();
    try {
      const { runtime, models } = await loadTraverseLiveRuntime(target);
      if (token !== startToken || !active) return false;
      // Same origin by default; CarrotTraverseLiveSocketBase (e.g. "ws://127.0.0.1:8790") is for a local preview / mock comma.
      const socketBase = typeof target.CarrotTraverseLiveSocketBase === "string" ? target.CarrotTraverseLiveSocketBase : undefined;
      view = runtime.createView({ root: host, models, target, socketBase });
      view.start();
      return true;
    } catch (caught) {
      error = uiText(target, "traverse_live_load_failed", "Could not load Traverse Live.");
      target.console?.warn?.("[traverse live] start failed", caught);
      return false;
    } finally {
      if (token === startToken) loading = false;
      render();
    }
  }

  function stop() {
    startToken += 1;
    loading = false;
    if (view) {
      try { view.destroy(); } catch (caught) { target.console?.warn?.("[traverse live] stop failed", caught); }
      view = null;
    }
    render();
  }

  function requestStart() {
    if (active) return start();
    pendingAutoStart = true;
    return false;
  }

  startButton.addEventListener("click", () => { start(); });
  closeButton.addEventListener("click", () => {
    stop();
    // Opened from the vision start screen: give the slot back to Carrot Vision.
    const runtime = workspaceRuntime(target);
    if (runtime?.contentReplacement?.()?.to === TRAVERSE_LIVE_CONTENT_ID) runtime.restoreContent?.();
  });
  target.addEventListener?.("carrot:languagechange", render);
  render();

  const content = driveContentApi.create(TRAVERSE_LIVE_CONTENT_ID, {
    mount(nextHost) {
      if (!nextHost || typeof nextHost.appendChild !== "function") {
        throw new TypeError("Traverse Live content requires a slot host");
      }
      if (root.parentNode !== nextHost) nextHost.appendChild(root);
      mountedHost = nextHost;
    },
    activate() {
      active = true;
      if (pendingAutoStart) {
        pendingAutoStart = false;
        start();
      }
      return true;
    },
    deactivate() {
      active = false;
      // Off screen: close the socket so the comma stops streaming for this view.
      stop();
      return true;
    },
    resize() {
      view?.resize?.();
      return Boolean(view);
    },
    status() {
      return {
        mountedHostId: String(mountedHost?.id || ""),
        loading,
        error,
        view: view?.status?.() || null,
      };
    },
    destroy() {
      stop();
      target.removeEventListener?.("carrot:languagechange", render);
      root.remove();
      mountedHost = null;
      if (contentSingletons.get(target)?.content === content) contentSingletons.delete(target);
    },
  });
  if (!content) return null;
  contentSingletons.set(target, { content, requestStart });
  return content;
}

export function getOrCreateTraverseLiveContent(options = {}) {
  const target = options.target || globalThis;
  const existing = contentSingletons.get(target);
  if (existing) return existing.content;
  return createTraverseLiveContent({ ...options, target });
}

export function createTraverseLiveContentFactory(defaultOptions = {}) {
  return function traverseLiveContentFactory(context = {}) {
    const target = defaultOptions.target || globalThis;
    return getOrCreateTraverseLiveContent({ ...defaultOptions, target, context });
  };
}

export function installTraverseLiveContentGlobal(target = globalThis, content = null) {
  target.DriveContentTraverseLive = content;
  return content;
}

/* The "Traverse Live" button next to "Carrot Vision" on the drive start screen. */
export function installTraverseLiveLauncher(target = globalThis) {
  const doc = target.document;
  if (!doc || target.__carrotTraverseLiveLauncher) return false;
  target.__carrotTraverseLiveLauncher = true;
  injectStyle(doc);
  const label = () => {
    const button = doc.getElementById(TRAVERSE_LIVE_START_BUTTON_ID);
    if (button) button.textContent = `▶ ${uiText(target, "start_traverse_live", "Start Traverse Live")}`;
  };
  doc.addEventListener("click", (event) => {
    const button = event.target?.closest?.(`#${TRAVERSE_LIVE_START_BUTTON_ID}`);
    if (!button) return;
    event.preventDefault();
    requestTraverseLiveInPlaceOfVision(target);
  });
  target.addEventListener?.("carrot:languagechange", label);
  if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", label, { once: true });
  else label();
  return true;
}
