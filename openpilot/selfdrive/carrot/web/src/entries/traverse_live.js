// Traverse Live renderer (three.js scene + HUD), loaded on demand by the drive tab's traverse_live content.
import { createTraverseLiveView } from "../features/drive/contents/traverse_live/view.js";

globalThis.CarrotTraverseLive = Object.freeze({ version: 1, createView: createTraverseLiveView });
