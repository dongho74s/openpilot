// Read actual closure counters via CDP; never print tokens, coordinates or frames.
const targets=await(await fetch('http://127.0.0.1:9339/json/list')).json();
const target=targets.find(t=>t.url==='file:///android_asset/main.html');
const ws=new WebSocket(target.webSocketDebuggerUrl),pending=new Map();let seq=0;
await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject});
ws.onmessage=e=>{const v=JSON.parse(e.data);if(v.id){const p=pending.get(v.id);pending.delete(v.id);v.error?p.reject(v.error):p.resolve(v.result)}};
const send=(method,params={})=>new Promise((resolve,reject)=>{const id=++seq;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params}))});
try {
  const fn=await send('Runtime.evaluate',{expression:'window.stopWayonLiveView'});
  const props=await send('Runtime.getProperties',{objectId:fn.result.objectId});
  const scopes=props.internalProperties.find(p=>p.name==='[[Scopes]]');
  const items=await send('Runtime.getProperties',{objectId:scopes.value.objectId});
  for(const item of items.result.filter(p=>/^\d+$/.test(p.name))) {
    const p=await send('Runtime.getProperties',{objectId:item.value.objectId});
    const streams=p.result.find(v=>v.name==='streams');
    if(!streams)continue;
    const sock=p.result.find(v=>v.name==='socket');
    if(sock?.value?.objectId){
      const s=await send('Runtime.callFunctionOn',{objectId:sock.value.objectId,returnByValue:true,functionDeclaration:'function(){return {readyState:this.readyState,bufferedAmount:this.bufferedAmount}}'});
      console.log(JSON.stringify({socket:s.result.value}));
    }
    const r=await send('Runtime.callFunctionOn',{objectId:streams.value.objectId,returnByValue:true,
      functionDeclaration:'function(){return Object.fromEntries(Object.entries(this).map(([k,s])=>[k,{ready:s.ready,keySeen:s.keySeen,queue:s.decoder?.decodeQueueSize,decoder:s.decoder?.state,timestamp:s.lastTimestamp,frameTimestamp:s.frame?.timestamp,received:s.frameReceivedAt}]))}'});
    console.log(JSON.stringify({streams:r.result.value,counters:Object.fromEntries(p.result.filter(v=>['frameCounter','measuredFps','animationFrame','lastRenderAt'].includes(v.name)).map(v=>[v.name,v.value.value]))}));
    const gl=p.result.find(v=>v.name==='glState');
    if(gl?.value?.objectId){
      const g=await send('Runtime.callFunctionOn',{objectId:gl.value.objectId,returnByValue:true,
        functionDeclaration:'function(){let gl=this.gl;return {error:gl.getError(),lost:gl.isContextLost(),size:[gl.drawingBufferWidth,gl.drawingBufferHeight],ready:[gl.getUniform(this.program,this.uniforms.uWideReady),gl.getUniform(this.program,this.uniforms.uDriverReady)]}}'});
      console.log(JSON.stringify({gl:g.result.value}));
    }
  }
} finally {ws.close()}
