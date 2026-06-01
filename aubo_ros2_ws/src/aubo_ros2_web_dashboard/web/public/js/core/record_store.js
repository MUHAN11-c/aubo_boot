// record_store.js — 监控快照记录 persistent store (localStorage)
// 遵循 utils.js 模式: 纯函数 + 具名导出
//
// 用法:
//   import { loadRecords, saveRecords, clearRecords } from './core/record_store.js';
//   const records = loadRecords();
//   records.push({ name: '快照 #1', ... });
//   saveRecords(records);

const KEY = 'ivg_monitor_records';

export function loadRecords() {
  try { return JSON.parse(localStorage.getItem(KEY)) || []; }
  catch (_) { return []; }
}

export function saveRecords(arr) {
  try { localStorage.setItem(KEY, JSON.stringify(arr)); return true; }
  catch (e) { return false; }
}

export function clearRecords() { localStorage.removeItem(KEY); }
