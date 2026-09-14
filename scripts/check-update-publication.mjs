// Validate the public feed against both the local APK and its immutable Git blob.
// No upload, auth lookup, signing key or network request is performed here.
import fs from 'node:fs';
import crypto from 'node:crypto';
import {execFileSync} from 'node:child_process';
import assert from 'node:assert/strict';
const root = new URL('../', import.meta.url);
const feed = JSON.parse(fs.readFileSync(new URL('downloads/update.json',root),'utf8'));
assert.equal(feed.schemaVersion,1);assert.equal(feed.channel,'hylink-dev');
assert.equal(feed.packageName,'app.hylink.mobile.debug');
assert.ok(Number.isSafeInteger(feed.versionCode)&&feed.versionCode>0);
assert.ok(Number.isSafeInteger(feed.size)&&feed.size>0&&feed.size<=80*1024*1024);
assert.match(feed.sha256,/^[0-9a-f]{64}$/);
const match = /^https:\/\/raw\.githubusercontent\.com\/leehyuk1108\/carrotpilot\/([0-9a-f]{40})\/(downloads\/hylink-dev-\d+\.\d+\.\d+\.apk)$/.exec(feed.apkUrl);
assert.ok(match,'APK must name an immutable carrotpilot commit');
const file = fs.readFileSync(new URL(match[2],root));
const blob = execFileSync('git',['show',`${match[1]}:${match[2]}`],{cwd:root,maxBuffer:81*1024*1024});
for (const bytes of [file,blob]) {
  assert.equal(bytes.length,feed.size);
  assert.equal(crypto.createHash('sha256').update(bytes).digest('hex'),feed.sha256);
}
const gradle = fs.readFileSync(new URL('app/build.gradle.kts',root),'utf8');
assert.equal(Number(/versionCode\s*=\s*(\d+)/.exec(gradle)[1]),feed.versionCode);
assert.equal(/versionName\s*=\s*"([^"]+)"/.exec(gradle)[1]+'-debug',feed.versionName);
console.log(`PASS immutable carrotpilot feed, APK bytes/hash and versionCode ${feed.versionCode}`);
