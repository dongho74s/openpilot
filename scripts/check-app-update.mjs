// Offline UI state tests. Does not contact GitHub, Cloud or a phone.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const root = new URL('../app/src/main/', import.meta.url);
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {textContent:'', hidden:false, disabled:false, dataset:{}, attributes:{},
    setAttribute(k,v){this.attributes[k]=v}, removeAttribute(k){delete this.attributes[k]},
    addEventListener(k,v){this[k]=v}});
  return elements.get(id);
}
const calls=[];
const context=vm.createContext({window:{Android:{checkAppUpdate(){calls.push('check')},downloadAppUpdate(){calls.push('download')},
  installAppUpdate(){calls.push('install')},cancelAppUpdate(){calls.push('cancel')}}},
  document:{getElementById:element,querySelector:()=>element('vehicle-tab')}});
vm.runInContext(fs.readFileSync(new URL('assets/hylink-update.js',root),'utf8'),context);
const send=value=>context.window.onHylinkUpdateState({currentVersion:'1.6.5-app-updates-debug',...value});
for(const state of ['idle','current','error']) {send({state});element('app-update-button').click();assert.equal(calls.at(-1),'check')}
send({state:'available',hasUpdate:true,latestVersion:'1.6.6-debug',notes:'<img src=x onerror=alert(1)>'});
element('app-update-button').click();assert.equal(calls.at(-1),'download');
assert.equal(element('app-update-badge').hidden,false);
assert.equal(element('app-update-notes').textContent,'<img src=x onerror=alert(1)>','Notes are text, never HTML');
for(const state of ['ready','permission']) {send({state});element('app-update-button').click();assert.equal(calls.at(-1),'install')}
for(const state of ['checking','downloading','verifying','installing']) {
  send({state,progress:42});assert.equal(element('app-update-button').disabled,true);
  assert.equal(element('app-update-cancel').hidden,state==='installing');
}
send({state:'downloading',progress:42});assert.equal(element('app-update-progress').value,42);
element('app-update-cancel').click();assert.equal(calls.at(-1),'cancel');
send({state:'current',hasUpdate:false});assert.equal(element('app-update-badge').hidden,true);
context.window.Android={};element('app-update-button').click();assert.match(element('app-update-status').textContent,/Android/);
const html=fs.readFileSync(new URL('assets/main.html',root),'utf8');
assert.ok(html.indexOf('id="app-update-card"')>html.indexOf('id="terminal-launch-caption"'));
assert.match(html,/id="app-update-status" role="status" aria-live="polite"/);
const manifest=fs.readFileSync(new URL('AndroidManifest.xml',root),'utf8');
assert.match(manifest,/REQUEST_INSTALL_PACKAGES/);assert.match(manifest,/android:exported="false"/);
assert.doesNotMatch(manifest,/android\.permission\.(UPDATE_PACKAGES_WITHOUT_USER_ACTION|INSTALL_PACKAGES)"/);
const provider=fs.readFileSync(new URL('res/xml/update_paths.xml',root),'utf8');
assert.match(provider,/path="app-updates\/"/);assert.doesNotMatch(provider,/<root-path|<external-path|path="\."/);
console.log('PASS update actions, busy/cancel/progress, text-only notes, tab badge, bottom placement and limited installer permissions');
