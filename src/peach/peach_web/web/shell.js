"use strict";

// Shared header/navigation/footer + accessible confirm + toast host.
// Pages keep their own content markup; this module owns only application
// chrome so five entry points cannot drift apart again.
(function (root) {
  const {safe, ageLabel, timeLabel} = root.Peach.ui;
  const PAGES = Object.freeze([
    {key: "overview", href: "/overview.html", label: "总览", index: "00"},
    {key: "monitor", href: "/monitor.html", label: "过程监控", index: "01"},
    {key: "debug", href: "/debug.html", label: "单步调试", index: "02"},
    {key: "analysis", href: "/analysis.html", label: "数据分析", index: "03"},
    {key: "force", href: "/force.html", label: "五点力", index: "04"},
  ]);
  const PATH_TO_KEY = Object.freeze({
    "/": "overview",
    "/overview.html": "overview",
    "/index.html": "monitor",
    "/monitor.html": "monitor",
    "/debug.html": "debug",
    "/analysis.html": "analysis",
    "/force.html": "force",
  });
  let mounted = false;
  let unsubscribeFeed = null;
  let dialogActive = false;
  let toastHost = null;
  let heightObserver = null;

  function notify(message, options = {}) {
    const tone = options.tone || "info";
    const timeout = options.timeout ?? 6000;
    if (!root.document?.body) return;
    if (!toastHost) {
      toastHost = document.createElement("div");
      toastHost.className = "app-toast-host";
      toastHost.id = "app-toasts";
      toastHost.setAttribute("aria-live", "polite");
      toastHost.setAttribute("aria-atomic", "false");
      document.body.appendChild(toastHost);
    }
    const item = document.createElement("div");
    item.className = `app-toast ${tone}`;
    item.setAttribute("role", tone === "err" ? "alert" : "status");
    item.textContent = String(message ?? "");
    toastHost.appendChild(item);
    if (timeout > 0) {
      setTimeout(() => {
        item.style.opacity = "0";
        setTimeout(() => item.remove(), 180);
      }, timeout);
    }
    return item;
  }

  function confirmDialog(options = {}) {
    const {
      title = "确认操作",
      message = "",
      confirmText = "确认",
      cancelText = "取消",
      danger = false,
    } = options;
    if (!root.document?.body) return Promise.resolve(false);
    if (dialogActive) {
      notify("已有确认窗口未关闭，请先处理。", {tone: "warn"});
      return Promise.resolve(false);
    }
    dialogActive = true;
    return new Promise((resolve) => {
      const dialog = document.createElement("dialog");
      dialog.className = "app-dialog";
      dialog.innerHTML = `<form method="dialog">
        <h2>${safe(title)}</h2>
        <p>${safe(message)}</p>
        <div class="app-dialog-actions">
          <button type="button" value="cancel" data-dialog-cancel>${safe(cancelText)}</button>
          <button type="submit" value="confirm" class="${danger ? "danger" : "primary"}" data-dialog-confirm>${safe(confirmText)}</button>
        </div>
      </form>`;
      let settled = false;
      const finish = (value) => {
        if (settled) return;
        settled = true;
        dialogActive = false;
        dialog.remove();
        resolve(value);
      };
      dialog.querySelector("[data-dialog-cancel]").addEventListener("click", () => {
        finish(false);
        dialog.close();
      });
      dialog.querySelector("[data-dialog-confirm]").addEventListener("click", (event) => {
        event.preventDefault();
        finish(true);
        dialog.close();
      });
      dialog.addEventListener("cancel", (event) => {
        event.preventDefault();
        finish(false);
      });
      dialog.addEventListener("close", () => finish(false));
      document.body.appendChild(dialog);
      dialog.showModal();
      // Safety default: focus the non-destructive choice first.
      dialog.querySelector("[data-dialog-cancel]").focus();
    });
  }

  function inferActive() {
    return PATH_TO_KEY[root.location?.pathname] || "";
  }

  function connectionText(status, describe) {
    if (typeof describe === "function") return describe(status);
    if (!status || status.state === "idle") {
      return {label: "静态记录", detail: "无实时轮询"};
    }
    if (status.state === "offline") {
      return {label: "连接中断", detail: status.error || "保留最后一次快照"};
    }
    if (status.state === "stale") {
      return {label: "数据陈旧", detail: `${ageLabel(status.ageS)} · 普通操作已暂停`};
    }
    if (status.state === "connecting") return {label: "正在连接", detail: "等待首个快照"};
    return {
      label: "已连接",
      detail: status.lastSuccessAt ? `更新于 ${timeLabel(status.lastSuccessAt / 1000)}` : "实时",
    };
  }

  function renderConnection(status) {
    const node = document.getElementById("app-connection");
    if (!node) return;
    const text = connectionText(status, renderConnection.describe);
    node.dataset.state = status?.state || "idle";
    node.querySelector("span").textContent = text.label;
    const transport = {http: "HTTP", rosbridge: "ROSbridge"}[root.PeachRosbridge?.mode];
    node.querySelector("small").textContent = transport ? `${text.detail} · ${transport}` : text.detail;
    node.title = status?.error || text.detail;
  }

  function syncHeaderHeight() {
    const header = document.getElementById("app-header");
    const context = document.querySelector(".page-context");
    if (!header) return;
    const height = header.getBoundingClientRect().height
      + (context ? context.getBoundingClientRect().height : 0);
    document.documentElement.style.setProperty("--header-height", `${Math.round(height)}px`);
    document.documentElement.style.setProperty("--app-header-height", `${Math.round(height)}px`);
  }

  function ensureStructure(active, title, eyebrow) {
    if (!document.getElementById("skip-link")) {
      const skip = document.createElement("a");
      skip.id = "skip-link";
      skip.className = "skip-link";
      skip.href = "#main";
      skip.textContent = "跳到主内容";
      document.body.prepend(skip);
    }
    let header = document.getElementById("app-header");
    if (!header) {
      header = document.createElement("header");
      header.id = "app-header";
      document.body.prepend(header);
    }
    const links = PAGES.map((page) => {
      const current = page.key === active ? ' aria-current="page"' : "";
      const mode = new URLSearchParams(root.location.search).get("transport");
      const href = ["http", "rosbridge"].includes(mode) ? `${page.href}?transport=${mode}` : page.href;
      return `<a href="${href}"${current}><span>${page.label}</span><small>${page.index}</small></a>`;
    }).join("");
    header.className = "app-header";
    header.innerHTML = `<div class="app-header-inner">
      <a class="app-brand" href="/overview.html" aria-label="Peach 系统总览">
        <span class="app-brand-mark" aria-hidden="true"><span class="fruit"></span><span class="leaf"></span><span class="pip"></span></span>
        <span class="app-brand-text"><span class="app-eyebrow">${safe(eyebrow)}</span><span class="app-title">${safe(title)}</span></span>
      </a>
      <nav class="app-nav" aria-label="主导航">${links}</nav>
      <div class="app-conn" id="app-connection" data-state="idle" role="status" aria-live="polite">
        <i aria-hidden="true"></i><span>静态记录</span><small>无实时轮询</small>
      </div>
    </div>`;
    const main = document.querySelector("main");
    if (main && main.id !== "main") main.id = "main";
  }

  function mount(options = {}) {
    const {
      active = inferActive(),
      title = "Peach",
      eyebrow = "PEACH / OPERATIONS",
      feed = null,
      connection = null,
      describe = null,
      footer = null,
    } = options;
    if (!root.document?.body) return null;
    ensureStructure(active, title, eyebrow);
    mounted = true;
    document.body.classList.add("shell");
    if (unsubscribeFeed) { unsubscribeFeed(); unsubscribeFeed = null; }
    renderConnection.describe = describe;
    if (feed) {
      unsubscribeFeed = feed.subscribe(renderConnection);
    } else {
      renderConnection(connection || {state: "idle"});
    }
    const transportMode = new URLSearchParams(root.location.search).get("transport");
    if (["http", "rosbridge"].includes(transportMode)) {
      document.querySelectorAll("a[href]").forEach((link) => {
        const target = new URL(link.href, root.location.href);
        if (target.origin === root.location.origin && PATH_TO_KEY[target.pathname]) {
          target.searchParams.set("transport", transportMode);
          link.href = target.href;
        }
      });
    }
    const footerNode = document.getElementById("app-footer");
    if (footerNode && footer) {
      const parts = Array.isArray(footer) ? footer : [footer];
      footerNode.className = "app-footer";
      footerNode.innerHTML = parts.map((part) => `<span>${safe(part)}</span>`).join("");
    } else if (footerNode && !footerNode.textContent.trim()) {
      footerNode.className = "app-footer";
      footerNode.innerHTML = "<span>Peach · 8090 · 只读监控与显式调试；不替代硬件急停。</span><span>操作前核对现场与使能。</span>";
    }
    if (!heightObserver) {
      const targets = [document.getElementById("app-header"), document.querySelector(".page-context")].filter(Boolean);
      if (root.ResizeObserver) {
        heightObserver = new ResizeObserver(syncHeaderHeight);
        targets.forEach((node) => heightObserver.observe(node));
      }
      root.addEventListener("resize", syncHeaderHeight);
    }
    syncHeaderHeight();
    if (document.fonts?.ready) document.fonts.ready.then(syncHeaderHeight).catch(() => {});
    return {mounted, syncHeaderHeight};
  }

  function setConnection(value) {
    renderConnection.describe = null;
    renderConnection(value);
  }

  function setFooter(parts) {
    const node = document.getElementById("app-footer");
    if (!node) return;
    node.className = "app-footer";
    node.innerHTML = (Array.isArray(parts) ? parts : [parts])
      .map((part) => `<span>${safe(part)}</span>`).join("");
  }

  root.PeachShell = Object.freeze({
    PAGES, mount, notify, confirm: confirmDialog, setConnection, setFooter, syncHeaderHeight,
  });
})(typeof window !== "undefined" ? window : globalThis);
