"use strict";

/* Styles for the Carrot Vision "new ui 1" (C3 v10-nui) HUD. Injected once by the
 * overlay. Sizes are in C3 design units: --cnui-u is one unit in CSS pixels
 * (view height / 1080), set by the overlay on every relayout. */

export const NEWUI_STYLE_ID = "carrot-newui-hud-style";
export const NEWUI_FONT_FAMILY = "CarrotC3Pretendard";
export const NEWUI_ASSET_ROOT = "/shared-assets";

export const NEWUI_CSS = `
@font-face{
  font-family:"${NEWUI_FONT_FAMILY}";
  src:url("${NEWUI_ASSET_ROOT}/fonts/Pretendard-SemiBold.ttf") format("truetype");
  font-display:swap;
  font-style:normal;
  font-weight:600;
}
/* The classic HUD card stays only as a lifecycle root; this overlay draws. */
#driveHudCard { display:none !important; }
/* The mini HUD is its own feature: hide it unless that mode is on. */
html:not([data-carrot-mini-hud="1"]) #carrotMiniHud { display:none !important; }
/* While this overlay shows, it also draws the alerts the C3 way. */
#carrotStage.has-cnui #carrotOnroadAlert { display:none !important; }

/* v10-nui camera filter: disengaged and not steering is grey and 62 % bright. */
#carrotStage[data-cnui-tone="dim"] .carrot-stage__video,
#carrotStage[data-cnui-tone="dim"] .carrot-stage__videoHold,
#carrotStage[data-cnui-tone="dim"] .carrot-stage__replayViewport{
  filter:grayscale(1) brightness(.62);
}

/* view.py gradients, drawn under the lanes and path (same layer as the video). */
.cnui-under{
  position:absolute; pointer-events:none; z-index:var(--drive-layer-media, 1);
  left:var(--carrot-viewport-left, 0px);
  top:var(--carrot-viewport-top, 0px);
  width:var(--carrot-viewport-width, 100%);
  height:var(--carrot-viewport-height, 100%);
}
.cnui-under[hidden]{display:none!important}
.cnui-under > div{position:absolute; left:0; right:0}
.cnui-under-chrome{top:0; height:55%;
  background:linear-gradient(to bottom, rgba(14,28,41,${(0x34 / 255).toFixed(3)}) 0%, rgba(18,42,59,${(0x18 / 255).toFixed(3)}) 30%, rgba(18,42,59,0) 100%)}
.cnui-under-footer{top:56%; height:44%;
  background:linear-gradient(to bottom, rgba(5,9,15,0) 0%, rgba(5,9,15,${(0x34 / 255).toFixed(3)}) 35%, rgba(5,9,15,${(0x92 / 255).toFixed(3)}) 72%, rgba(5,9,15,${(0xD8 / 255).toFixed(3)}) 100%)}
.cnui-under-hud{top:66%; height:34%;
  background:linear-gradient(to bottom, rgba(3,5,9,0) 0%, rgba(3,5,9,${(0x30 / 255).toFixed(3)}) 22%, rgba(3,5,9,${(0x88 / 255).toFixed(3)}) 58%, rgba(3,5,9,${(0xE8 / 255).toFixed(3)}) 100%)}

.cnui{
  position:absolute; pointer-events:none; z-index:7; isolation:isolate; overflow:hidden;
  left:var(--carrot-viewport-left, 0px);
  top:var(--carrot-viewport-top, 0px);
  width:var(--carrot-viewport-width, 100%);
  height:var(--carrot-viewport-height, 100%);
  --cnui-u:1px;
  font-family:"${NEWUI_FONT_FAMILY}", "Wanted Sans Variable", "Pretendard", system-ui, sans-serif;
  font-weight:600;
}
.cnui[hidden],.cnui [hidden]{display:none!important}
.cnui-svg{position:absolute; inset:0; width:100%; height:100%; z-index:1; overflow:visible; display:block}
.cnui-svg text{font-family:inherit; font-weight:600}
.cnui-svg image{image-rendering:auto}

/* Full-screen alert text wraps like the C3 Label. */
.cnui-full{position:absolute; left:0; right:0; z-index:2; color:#fff; text-align:center;
  white-space:pre-wrap; overflow-wrap:anywhere; word-break:keep-all}

/* The C3 screen corners (radius 60 of a 1080 screen). */
.cnui-corners{
  position:absolute; inset:0; z-index:3;
  border-radius:calc(var(--cnui-u) * 60);
  box-shadow:0 0 0 calc(var(--cnui-u) * 400) #000;
}
`;

export function injectNewUiStyle(doc) {
  if (!doc || doc.getElementById?.(NEWUI_STYLE_ID)) return;
  const style = doc.createElement("style");
  style.id = NEWUI_STYLE_ID;
  style.textContent = NEWUI_CSS;
  (doc.head || doc.documentElement).appendChild(style);
}
