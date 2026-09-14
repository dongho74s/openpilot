import assert from 'node:assert/strict';
import fs from 'node:fs';
const root=new URL('../app/src/main/assets/',import.meta.url);
const html=fs.readFileSync(new URL('main.html',root),'utf8');
const live=fs.readFileSync(new URL('wayon_live.js',root),'utf8');
for (const name of ['hylink.js','hylink-app.js']) {
  const source=fs.readFileSync(new URL(name,root),'utf8');
  for (const removed of ['wayon-live-capture-saved','btnWayonLivePhoto','btnWayonLiveClip10','btnWayonLiveClip30']) {
    assert.ok(!source.includes(removed),name+': stale capture handler or label '+removed);
  }
}
for(const removed of ['btnWayonLivePhoto','btnWayonLiveClip10','btnWayonLiveClip30','wayon-live-capture-bar','wayon-live-save-toast']) {
  assert.ok(!html.includes(removed),removed+' must be removed, not hidden');
}
for(const removed of ['sendClientControl','capturePanoramaPhoto','requestClip','CONTROL_PHOTO','CONTROL_CLIP','showSaveToast','handleCaptureStatus']) {
  assert.ok(!live.includes(removed),removed+' must not remain executable');
}
for(const kept of ['btnWayonLive','photos-button','wayon-live-playback-bar'])assert.ok(html.includes(kept));
assert.ok(live.includes('function parseStoredZip('));
assert.ok(live.includes('window.startWayonSavedClip'));
console.log('View-only live: capture UI and send paths removed; live, automatic-photo viewing and old-clip playback preserved');
