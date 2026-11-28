import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

const toolsRuntimeSource = await readFile(new URL("../src/features/tools/runtime.js", import.meta.url), "utf8");
const indexHtml = await readFile(new URL("../index.html", import.meta.url), "utf8");

class FakeElement {
  constructor(tag) {
    this.tag = tag;
    this.children = [];
    this.dataset = {};
    this.hidden = false;
    this.className = "";
    this._text = "";
    this.classList = { toggle() {} };
  }
  appendChild(child) { this.children.push(child); return child; }
  removeAttribute() {}
  set textContent(value) { this._text = String(value); if (value === "") this.children = []; }
  get textContent() { return this._text + this.children.map((c) => c.textContent).join("|"); }
  find(tag) { return this.tag === tag ? this : this.children.map((c) => c.find?.(tag)).find(Boolean); }
  all(tag) { return [...(this.tag === tag ? [this] : []), ...this.children.flatMap((c) => c.all(tag))]; }
}

function loadRender(lang) {
  const elements = new Map();
  const document = {
    getElementById(id) {
      if (!elements.has(id)) elements.set(id, new FakeElement("host"));
      return elements.get(id);
    },
    createElement(tag) { return new FakeElement(tag); },
  };
  const strings = {
    tools_update_notes_title: "이번 업데이트 내용",
    tools_update_notes_more: "외 {count}개",
    tools_update_notes_undescribed: "설명 없는 변경 {count}개",
  };
  const getUIText = (key, fallback, vars) => {
    let text = lang === "ko" ? (strings[key] ?? fallback) : fallback;
    for (const [name, value] of Object.entries(vars || {})) text = text.replace(`{${name}}`, value);
    return text;
  };
  const start = toolsRuntimeSource.indexOf("function renderGitPullStatus(");
  const end = toolsRuntimeSource.indexOf("async function refreshGitPullStatus(", start);
  const render = new Function("document", "getUIText", "LANG",
    `${toolsRuntimeSource.slice(start, end)}; return renderGitPullStatus;`)(document, getUIText, lang);
  return { render, notesBox: () => elements.get("gitPullNotes") };
}

const notes = {
  items: [{ ko: "볼트 정지 유지", en: "Bolt holds a stop", commits: 4 }, { ko: "테마 버튼", en: "", commits: 1 }],
  more_items: 2,
  undescribed: [{ hash: "eeeeeeee", subject: "Fix a typo" }],
  undescribed_count: 3,
};

test("the update row lists the Korean notes, the rest under a fold with their subjects", () => {
  const { render, notesBox } = loadRender("ko");
  render({ state: "ok", behind: 8, notes });
  const box = notesBox();
  assert.equal(box.hidden, false);
  assert.equal(box.find("div").textContent, "이번 업데이트 내용");
  const list = box.children.find((c) => c.className === "git-pull-notes__list");
  assert.deepEqual(list.children.map((li) => li.textContent), ["볼트 정지 유지", "테마 버튼", "외 2개"]);
  const fold = box.find("details");
  assert.equal(fold.find("summary").textContent, "설명 없는 변경 3개");
  assert.deepEqual(fold.find("ul").children.map((li) => li.textContent), ["Fix a typo"]);
});

test("English shows the English note and falls back to Korean", () => {
  const { render, notesBox } = loadRender("en");
  render({ state: "ok", behind: 8, notes });
  const list = notesBox().children.find((c) => c.className === "git-pull-notes__list");
  assert.deepEqual(list.children.slice(0, 2).map((li) => li.textContent), ["Bolt holds a stop", "테마 버튼"]);
});

test("no notes, no updates, an error or a wait hides the box; the same notes are not rebuilt", () => {
  const { render, notesBox } = loadRender("ko");
  render({ state: "ok", behind: 8, notes });
  const box = notesBox();
  const fold = box.find("details");
  fold.open = true;
  render({ state: "ok", behind: 8, notes });
  assert.equal(box.find("details"), fold, "polling keeps an opened fold");
  render({ state: "ok", behind: 0, notes });
  assert.equal(box.hidden, true);
  render({ state: "ok", behind: 3, notes: null });
  assert.equal(box.hidden, true);
  render({ state: "busy", behind: 3, notes });
  assert.equal(box.hidden, true);
  render({ state: "ok", behind: 3, auto_update: { status: "error" }, notes });
  assert.equal(box.hidden, true);
});

test("the notes box sits right under the update row and text is never parsed as HTML", () => {
  assert.match(indexHtml, /id="btnGitPullAction"[^\n]*\n\s*<\/button>\n\s*<div id="gitPullNotes" class="git-pull-notes" hidden/);
  const start = toolsRuntimeSource.indexOf("function renderGitPullNotes(");
  const end = toolsRuntimeSource.indexOf("async function refreshGitPullStatus(", start);
  assert.ok(start > 0 && end > start);
  assert.doesNotMatch(toolsRuntimeSource.slice(start, end), /innerHTML|insertAdjacentHTML/);
});
