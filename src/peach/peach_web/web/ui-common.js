"use strict";

// Shared, dependency-free display helpers. Every page may use the legacy
// globals below or the namespaced Peach.ui API; new code should prefer the
// namespace so module ownership stays explicit.

const $ = (id) => document.getElementById(id);
const $$ = (selector, root = document) => Array.from(root.querySelectorAll(selector));
const numeric = (value) => (typeof value === "number" || typeof value === "string" && value.trim() !== "") &&
  Number.isFinite(Number(value));
const fmt = (value, digits = 3, suffix = "") => numeric(value)
  ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const percent = (value) => numeric(value) ? `${(Number(value) * 100).toFixed(1)}%` : "—";
const safe = (value) => String(value ?? "—").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
})[char]);
const setText = (id, value) => {
  const el = $(id);
  if (el) el.textContent = value ?? "—";
};

function clamp(value, min, max) {
  const number = Number(value);
  return Number.isFinite(number) ? Math.min(max, Math.max(min, number)) : min;
}

function timeLabel(stampSeconds) {
  return numeric(stampSeconds) && Number(stampSeconds) > 0
    ? new Date(Number(stampSeconds) * 1000).toLocaleTimeString("zh-CN", {hour12: false})
    : "--:--:--";
}

function ageLabel(ageSeconds) {
  if (!numeric(ageSeconds)) return "—";
  const age = Math.max(0, Number(ageSeconds));
  if (age < 1) return `${age.toFixed(2)} s 前`;
  if (age < 60) return `${age.toFixed(1)} s 前`;
  return `${Math.floor(age / 60)} min 前`;
}

function debounce(fn, wait = 120) {
  let timer = 0;
  return (...args) => {
    clearTimeout(timer);
    timer = setTimeout(() => fn(...args), wait);
  };
}

function paintStyles(root) {
  if (!root) return;
  root.querySelectorAll("[data-width]").forEach((node) => {
    node.style.width = `${clamp(node.dataset.width, 0, 100)}%`;
  });
  root.querySelectorAll("[data-left]").forEach((node) => {
    node.style.left = `${clamp(node.dataset.left, 0, 100)}%`;
  });
  root.querySelectorAll("[data-flex]").forEach((node) => {
    node.style.flex = `0 0 ${clamp(node.dataset.flex, 0, 100)}%`;
  });
}

const Peach = (globalThis.Peach = globalThis.Peach || {});
Peach.ui = Object.freeze({
  $, $$, numeric, fmt, percent, safe, setText, clamp, timeLabel, ageLabel,
  debounce, paintStyles,
});
Peach.time = Object.freeze({now: () => Date.now(), timeLabel, ageLabel});
