'use strict';
const byId=id=>document.getElementById(id);
function showKey(data){
  if(!data.ready)return;
  byId('result').hidden=false;byId('key').value=data.key;
  for(const id of ['media','impact','remote'])byId(id).checked=data[id]===true;
  byId('consent').checked=data.enabled===true;
  byId('features').textContent=data.enabled?'차량 정보 연결됨'+(data.media?' · 라이브/주차 사진':'')+(data.impact?' · 충격 기록':'')+(data.remote?' · 원격 터미널':''):'정보 전송이 중지되어 있어요. 아래에서 다시 활성화하세요.';
}
async function loadKey(){
  try{const r=await fetch('/api/key',{headers:{'X-Hylink-Request':'1'},cache:'no-store'});const d=await r.json();if(!r.ok)throw new Error(d.error);showKey(d)}
  catch(e){byId('status').textContent=e.message||'키를 불러오지 못했어요. 새로고침해 주세요.'}
}
byId('connect').onclick=async()=>{
  if(!byId('consent').checked){byId('status').textContent='차량 정보 전송 동의가 필요해요.';byId('consent').focus();return}
  byId('connect').disabled=true;byId('status').textContent='클라우드에 차량을 연결하고 있어요…';
  try{
    const response=await fetch('/api/connect',{method:'POST',headers:{'Content-Type':'application/json','X-Hylink-Request':'1'},cache:'no-store',body:JSON.stringify({consent:true,media:byId('media').checked,impact:byId('impact').checked,remote:byId('remote').checked})});
    const data=await response.json();if(!response.ok)throw new Error(data.error);
    showKey(data);byId('status').textContent='설정 완료. 위의 키를 앱에 붙여 넣으면 연결돼요.';
    byId('copy').focus();
  }catch(error){byId('status').textContent=error.message||'연결이 끊겼어요. 다시 시도해 주세요.'}
  finally{byId('connect').disabled=false}
};
byId('copy').onclick=async()=>{
  try{if(navigator.clipboard&&window.isSecureContext)await navigator.clipboard.writeText(byId('key').value);
    else{byId('key').select();if(!document.execCommand('copy'))throw new Error('copy')}
    byId('status').textContent='복사했어요. Hylink 앱에서 붙여 넣어 주세요.';
  }catch{byId('key').select();byId('status').textContent='키를 길게 눌러 복사해 주세요.'}
};
window.addEventListener('pagehide',()=>{byId('key').value=''});
window.addEventListener('pageshow',event=>{if(event.persisted)loadKey()});
loadKey();
// An already-open page must also clear its displayed credential after ignition
// stops the key server. No telemetry/media polling is added to the vehicle.
setInterval(async()=>{
  if(document.hidden||byId('result').hidden)return;
  try{const r=await fetch('/api/key',{headers:{'X-Hylink-Request':'1'},cache:'no-store',signal:AbortSignal.timeout(2500)});if(!r.ok)throw new Error('offroad');
    const d=await r.json();if(!d.ready)throw new Error('unavailable');
  }catch{byId('key').value='';byId('result').hidden=true;byId('status').textContent='키 페이지 연결이 종료됐어요. 시동을 끄고 같은 Wi-Fi에서 새로고침해 주세요.'}
},5000);
