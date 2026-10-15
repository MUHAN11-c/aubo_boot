"use strict";

// Display formatting shared by monitor renderers and application orchestration.
const $ = (id) => document.getElementById(id);
const numeric = (value) => (typeof value === "number" || typeof value === "string" && value.trim() !== "") &&
  Number.isFinite(Number(value));
const fmt = (value, digits = 3, suffix = "") => numeric(value)
  ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const percent = (value) => numeric(value) ? `${(Number(value) * 100).toFixed(1)}%` : "—";
const safe = (value) => String(value ?? "—").replace(/[&<>"']/g, (char) => ({
  "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"
})[char]);
const setText = (id, value) => { const el = $(id); if (el) el.textContent = value ?? "—"; };
