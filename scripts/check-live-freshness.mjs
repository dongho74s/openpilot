import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const source=fs.readFileSync(new URL('../app/src/main/assets/wayon_live.js',import.meta.url),'utf8');
const start=source.indexOf('    function updateVideoFreshness(now) {');
const end=source.indexOf('    function startHeartbeat',start);
assert.ok(start>=0&&end>start);
let live=true,label='',messageVisible=false;
const streams={wide:{lastFrameAt:0},driver:{lastFrameAt:0}};
const ctx=vm.createContext({savedPlayback:null,terminalState:null,videoStalled:false,streams,
  status:{classList:{contains:()=>live}},
  setStatus(text,isLive=false){label=text;live=isLive},
  showMessage(){messageVisible=true},hideMessage(){messageVisible=false},
});
vm.runInContext(source.slice(start,end),ctx);
ctx.updateVideoFreshness(10000);assert.equal(ctx.videoStalled,false); // startup timer owns no-frame case
streams.wide.lastFrameAt=streams.driver.lastFrameAt=10000;
ctx.updateVideoFreshness(15000);assert.equal(ctx.videoStalled,false);
streams.wide.lastFrameAt=16000;
ctx.updateVideoFreshness(16000);assert.equal(ctx.videoStalled,true); // just one frozen camera is enough
assert.equal(label,'영상 수신 지연');assert.equal(live,false);assert.equal(messageVisible,true);
streams.driver.lastFrameAt=16000;
ctx.updateVideoFreshness(16000);assert.equal(ctx.videoStalled,false);assert.equal(live,true);assert.equal(messageVisible,false);
ctx.savedPlayback={paused:true};ctx.updateVideoFreshness(25000);assert.equal(ctx.videoStalled,false);
ctx.savedPlayback=null;ctx.terminalState='expired';ctx.updateVideoFreshness(25000);assert.equal(ctx.videoStalled,false);
assert.match(source,/stream\.lastFrameAt = stream\.frameReceivedAt/);
assert.match(source,/stream\.lastFrameAt = 0/);
console.log('Live freshness: startup, 5s boundary, one-camera stall, recovery, playback and terminal guards passed');
