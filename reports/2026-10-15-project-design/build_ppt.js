/* 桃采摘机器人项目设计 PPT 生成脚本 */
const pptxgen = require("pptxgenjs");
const React = require("react");
const ReactDOMServer = require("react-dom/server");
const sharp = require("sharp");
const fs = require("fs");
const path = require("path");
const Fa = require("react-icons/fa");

const ROOT = __dirname;
const ASSETS = path.join(ROOT, "assets");
const ICONS = path.join(ROOT, "icons");
if (!fs.existsSync(ICONS)) fs.mkdirSync(ICONS, { recursive: true });

// ---------- palette ----------
const C = {
  deep: "1E3B24",      // 封面/尾页深林绿
  forest: "2C5F2D",    // 主绿
  moss: "97BC62",      // 苔绿
  mossSoft: "C9DCAB",  // 浅苔
  peach: "E8A87C",     // 桃色点缀
  peachDeep: "C96F4A", // 深桃（强调文字）
  bg: "FFFFFF",
  tint: "F3F7F1",      // 浅底
  card: "FFFFFF",
  text: "22301F",
  mute: "66755F",
  line: "D9E4D4",
  white: "FFFFFF",
};
const TITLE_FONT = "Cambria";
const BODY_FONT = "Calibri";

// ---------- icons ----------
const ICON_DEFS = {
  bullseye: Fa.FaBullseye, users: Fa.FaUsers, layers: Fa.FaLayerGroup,
  sitemap: Fa.FaSitemap, chip: Fa.FaMicrochip, shield: Fa.FaShieldAlt,
  branch: Fa.FaCodeBranch, eye: Fa.FaEye, robot: Fa.FaRobot, cut: Fa.FaCut,
  vial: Fa.FaVial, tractor: Fa.FaTractor, cubes: Fa.FaCubes,
  chart: Fa.FaChartLine, swap: Fa.FaExchangeAlt, map: Fa.FaMapSigns,
  flag: Fa.FaFlagCheckered, camera: Fa.FaCamera, server: Fa.FaServer,
  box: Fa.FaBoxOpen, tasks: Fa.FaTasks, wrench: Fa.FaWrench,
  leaf: Fa.FaLeaf, route: Fa.FaRoute, database: Fa.FaDatabase,
  check: Fa.FaCheckCircle, ban: Fa.FaBan, heartbeat: Fa.FaHeartbeat,
  play: Fa.FaPlayCircle, bolt: Fa.FaBolt, book: Fa.FaBookOpen,
};
async function renderIcons() {
  for (const [name, Comp] of Object.entries(ICON_DEFS)) {
    const svg = ReactDOMServer.renderToStaticMarkup(
      React.createElement(Comp, { color: "#FFFFFF", size: 128 })
    );
    const buf = await sharp(Buffer.from(svg)).resize(128, 128).png().toBuffer();
    fs.writeFileSync(path.join(ICONS, `${name}.png`), buf);
  }
}
const icon = (name) => `image/png;base64,${fs.readFileSync(path.join(ICONS, `${name}.png`)).toString("base64")}`;

// ---------- helpers ----------
const PW = 13.333, PH = 7.5;
function titleBar(slide, kicker, title, pageNo) {
  slide.addText(kicker, {
    x: 0.55, y: 0.34, w: 9.5, h: 0.32, fontFace: BODY_FONT, fontSize: 12,
    color: C.peachDeep, bold: true, charSpacing: 2, margin: 0,
  });
  slide.addText(title, {
    x: 0.52, y: 0.62, w: 11.2, h: 0.75, fontFace: TITLE_FONT, fontSize: 30,
    color: C.text, bold: true, margin: 0,
  });
  slide.addText(String(pageNo).padStart(2, "0"), {
    x: PW - 1.05, y: 0.38, w: 0.55, h: 0.4, fontFace: BODY_FONT, fontSize: 12,
    color: C.mute, align: "right", margin: 0,
  });
}
function card(slide, x, y, w, h, opts = {}) {
  slide.addShape("roundRect", {
    x, y, w, h, rectRadius: 0.09,
    fill: { color: opts.fill || C.card },
    line: opts.noLine ? { type: "none" } : { color: opts.lineColor || C.line, width: 1 },
    shadow: opts.shadow ? { type: "outer", angle: 90, blur: 7, offset: 2, opacity: 0.16, color: "1E3B24" } : undefined,
  });
}
function iconCircle(slide, name, x, y, d, bg) {
  slide.addShape("ellipse", { x, y, w: d, h: d, fill: { color: bg || C.forest }, line: { type: "none" } });
  const pad = d * 0.26;
  slide.addImage({ data: icon(name), x: x + pad, y: y + pad, w: d - 2 * pad, h: d - 2 * pad });
}
function bullets(slide, items, x, y, w, h, opts = {}) {
  slide.addText(
    items.map((t, i) => ({
      text: t,
      options: {
        bullet: { code: "2022", indent: 14 },
        breakLine: i < items.length - 1,
        color: opts.color || C.text,
        bold: !!opts.bold,
      },
    })),
    {
      x, y, w, h, fontFace: BODY_FONT, fontSize: opts.size || 13,
      color: opts.color || C.text, paraSpaceAfter: opts.gap ?? 8,
      align: "left", valign: "top", margin: 0, lineSpacingMultiple: 1.12,
    }
  );
}
function statBox(slide, x, y, w, num, label, numColor) {
  slide.addText(num, { x, y, w, h: 0.62, fontFace: TITLE_FONT, fontSize: 30, bold: true, color: numColor || C.forest, align: "center", margin: 0 });
  slide.addText(label, { x, y: y + 0.6, w, h: 0.55, fontFace: BODY_FONT, fontSize: 10.5, color: C.mute, align: "center", valign: "top", margin: 0 });
}
function pic(slide, file, x, y, w, h) {
  slide.addImage({ path: path.join(ASSETS, file), x, y, w, h, rounding: false });
}
function picFit(slide, file, x, y, w, h) { // contain
  const meta = { "rviz_fullchain.png":[1400,900], "stereo_cmp.png":[1920,976], "masks_mid.png":[640,480],
    "percipio_view.png":[1228,866], "sim_detail.png":[1280,720], "sim_lighting.jpg":[1920,1950],
    "sim_lighting_wide.jpg":[1920,819],
    "web_console.png":[1440,900], "yolo_val.jpg":[1920,1100], "v2_topics.png":[1418,814] }[file];
  const [iw, ih] = meta, ar = iw / ih, boxAr = w / h;
  let dw = w, dh = h;
  if (ar > boxAr) dh = w / ar; else dw = h * ar;
  slide.addImage({ path: path.join(ASSETS, file), x: x + (w - dw) / 2, y: y + (h - dh) / 2, w: dw, h: dh });
}

async function main() {
  await renderIcons();
  const pres = new pptxgen();
  pres.layout = "LAYOUT_WIDE";
  pres.author = "peach 项目组";
  pres.title = "桃采摘机器人项目设计";

  // ============ S1 封面 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.deep };
    // 右侧半出血图
    s.addImage({ path: path.join(ASSETS, "sim_detail.png"), x: 7.7, y: 0, w: PW - 7.7, h: PH });
    s.addShape("rect", { x: 7.7, y: 0, w: PW - 7.7, h: PH, fill: { color: C.deep, transparency: 42 }, line: { type: "none" } });
    s.addText("PEACH · 自主采摘机器人", { x: 0.75, y: 1.15, w: 6.6, h: 0.4, fontFace: BODY_FONT, fontSize: 14, color: C.moss, bold: true, charSpacing: 3, margin: 0 });
    s.addText("桃套袋采摘机器人\n项目设计报告", { x: 0.7, y: 1.6, w: 7.0, h: 2.2, fontFace: TITLE_FONT, fontSize: 44, color: C.white, bold: true, margin: 0, lineSpacingMultiple: 1.05 });
    s.addText("自主导航 · 感知建模 · 剪切采摘 · 收集物流", { x: 0.75, y: 3.9, w: 6.8, h: 0.45, fontFace: BODY_FONT, fontSize: 16, color: C.peach, margin: 0 });
    s.addText("项目设计定稿 2026-10 · 工程演进复盘 2026-08-15 → 2026-10-15", { x: 0.75, y: 4.42, w: 6.8, h: 0.4, fontFace: BODY_FONT, fontSize: 12.5, color: "BFD3B8", margin: 0 });
    // 底部数据条
    const stats = [
      ["205", "笔提交（近两月）"], ["1.83M+", "行代码入库"], ["14", "工作区能力包"], ["60", "会话数据包 MCAP"],
    ];
    stats.forEach(([n, l], i) => {
      const x = 0.75 + i * 1.75;
      s.addText(n, { x, y: 5.55, w: 1.6, h: 0.55, fontFace: TITLE_FONT, fontSize: 26, bold: true, color: C.moss, margin: 0 });
      s.addText(l, { x, y: 6.08, w: 1.7, h: 0.6, fontFace: BODY_FONT, fontSize: 10, color: "BFD3B8", margin: 0 });
    });
    s.addNotes("封面：项目设计 + 近两月工程演进。数据源：git log --since 2026-08-15（205 提交、2558 文件、183 万行入库）；runs/ 60 个会话 mcap。");
  }

  // ============ S2 目录 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "CONTENTS", "目录", 2);
    const items = [
      ["book", "项目设计", "产品目标 · 用户与职责 · 12 模块架构 · 完整作业闭环"],
      ["chip", "技术栈与安全", "硬件与软件栈 · 工作区布局 · 三层安全与使能门"],
      ["eye", "感知与运动设计", "停走式感知管线 · 接近轨迹四代演进 · 执行命令门"],
      ["cut", "工具与测试体系", "三把剪切手 · 测试金字塔 · 真机验证记录"],
      ["cubes", "仿真体系", "src/sim 五包 · 统一机体 · 感知闭环 · 自主导航采摘"],
      ["swap", "演进数据与路线图", "两月提交曲线 · 优化前后对比 · G0-G4 建设路线"],
    ];
    items.forEach(([ic, t, d], i) => {
      const col = i % 2, row = Math.floor(i / 2);
      const x = 0.62 + col * 6.25, y = 1.78 + row * 1.72;
      card(s, x, y, 5.9, 1.5, { shadow: true });
      iconCircle(s, ic, x + 0.28, y + 0.4, 0.7, row === 2 ? C.peachDeep : C.forest);
      s.addText(`${i + 1}. ${t}`, { x: x + 1.18, y: y + 0.22, w: 4.5, h: 0.45, fontFace: TITLE_FONT, fontSize: 17, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: x + 1.18, y: y + 0.72, w: 4.55, h: 0.62, fontFace: BODY_FONT, fontSize: 11.5, color: C.mute, margin: 0 });
    });
    s.addNotes("六个部分：设计、栈与安全、感知运动、工具测试、仿真、数据与路线。");
  }

  // ============ S3 产品目标与用户 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "01 PROJECT BLUEPRINT", "产品目标：一条完整的移动采摘产品线", 3);
    // 左：目标卡
    card(s, 0.62, 1.7, 6.1, 3.1, { shadow: true });
    iconCircle(s, "bullseye", 0.92, 1.98, 0.62, C.forest);
    s.addText("目标定位", { x: 1.7, y: 2.02, w: 3.4, h: 0.5, fontFace: TITLE_FONT, fontSize: 16, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "在指定果园工作区完成自主导航、作业点定位、目标发现、选择性剪切采摘、承接收集与异常恢复",
      "基准工艺：套袋桃剪切采摘；首版单机产品线，多机协作仅留扩展边界",
      "完整范围：底盘、定位传感、近场视觉、机械臂、工具、承接容器、卸载/补能与操作台",
      "现场运动与恢复不依赖云或网页持续在线",
    ], 0.95, 2.62, 5.5, 2.1, { size: 12.5 });
    // 左下：五级工作对象
    card(s, 0.62, 5.05, 6.1, 1.85, { fill: C.tint, noLine: true });
    s.addText("五级工作对象", { x: 0.9, y: 5.22, w: 3.5, h: 0.4, fontFace: BODY_FONT, fontSize: 12.5, bold: true, color: C.forest, margin: 0 });
    const levels = ["果园任务", "行/分区", "作业点", "单果周期", "动作阶段"];
    levels.forEach((t, i) => {
      const x = 0.9 + i * 1.16;
      s.addShape("roundRect", { x, y: 5.68, w: 1.02, h: 0.52, rectRadius: 0.08, fill: { color: i === 4 ? C.peachDeep : C.forest }, line: { type: "none" } });
      s.addText(t, { x, y: 5.68, w: 1.02, h: 0.52, fontFace: BODY_FONT, fontSize: 10, color: C.white, align: "center", valign: "middle", margin: 0 });
      if (i < 4) s.addText("→", { x: x + 1.0, y: 5.68, w: 0.2, h: 0.52, fontFace: BODY_FONT, fontSize: 12, color: C.mute, align: "center", valign: "middle", margin: 0 });
    });
    s.addText("导航成功、剪切指令受理、采摘确认、入箱确认——四类结果不可互相替代", { x: 0.9, y: 6.36, w: 5.6, h: 0.4, fontFace: BODY_FONT, fontSize: 10.5, italic: true, color: C.peachDeep, margin: 0 });
    // 右：四类角色
    const roles = [
      ["users", "作业员", "设置区域与任务 · 核对现场 · 发起/暂停/取消 · 换箱与必要恢复"],
      ["wrench", "维护人员", "设备与标定 · 工具与地图站点维护 · 受控配置变更"],
      ["vial", "研发人员", "模块验证 · 仿真/回放 · 故障复现与结果对照"],
      ["tasks", "项目管理人员", "任务质量与遗漏 · 效率与资源 · 人工介入与验收进度"],
    ];
    roles.forEach(([ic, t, d], i) => {
      const y = 1.7 + i * 1.32;
      card(s, 7.0, y, 5.68, 1.14, { shadow: true });
      iconCircle(s, ic, 7.24, y + 0.25, 0.62, i % 2 ? C.peachDeep : C.forest);
      s.addText(t, { x: 8.02, y: y + 0.12, w: 4.4, h: 0.4, fontFace: TITLE_FONT, fontSize: 14.5, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: 8.02, y: y + 0.52, w: 4.5, h: 0.55, fontFace: BODY_FONT, fontSize: 10.5, color: C.mute, margin: 0 });
    });
    s.addNotes("总纲要点：单机产品线、套袋桃基准工艺、五级工作对象与四类不可替代结果。来源 peach_autonomous_harvest_design/01。");
  }

  // ============ S4 系统架构 12 模块 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "01 ARCHITECTURE", "系统架构：12 模块与责任边界", 4);
    const groups = [
      { name: "资源与调度", color: C.forest, mods: [
        ["M01", "果园与作业资源"], ["M02", "任务与全局调度"], ["M03", "定位与自主导航"], ["M04", "作业点定位与作业交接"]] },
      { name: "感知与建模", color: C.peachDeep, principle: "感知只产出事实（目标/位姿/质量）；模型把事实转成可操作条件（可达性/余量/许可）。感知不发运动、不选下一颗、不写账本。", mods: [
        ["M05", "场景感知与目标发现"], ["M06", "局部建模与操作条件评估"]] },
      { name: "执行与工具", color: C.forest, principle: "剪切、带离、承接、释放分别保留结果证据；结果未知进入受控恢复。", mods: [
        ["M07", "采摘策略与单果调度"], ["M08", "机械臂规划与运动执行"], ["M09", "末端工具与采摘验证"]] },
      { name: "支撑与闭环", color: C.peachDeep, principle: "任务/工作点/单果/物流状态不混写；Web 表达意图不承载权威。", mods: [
        ["M10", "收集物流与资源管理"], ["M11", "运行权限、健康与恢复"], ["M12", "数据闭环与 Web 工作台"]] },
    ];
    const gw = 3.0, gx0 = 0.62, gy = 1.72, gh = 4.1;
    groups.forEach((g, gi) => {
      const x = gx0 + gi * (gw + 0.18);
      card(s, x, gy, gw, gh, { fill: C.tint, noLine: true });
      s.addText(g.name, { x: x + 0.2, y: gy + 0.16, w: gw - 0.4, h: 0.42, fontFace: TITLE_FONT, fontSize: 14.5, bold: true, color: g.color, margin: 0 });
      g.mods.forEach(([code, name], mi) => {
        const y = gy + 0.68 + mi * 0.86;
        s.addShape("roundRect", { x: x + 0.18, y, w: gw - 0.36, h: 0.74, rectRadius: 0.07, fill: { color: C.white }, line: { color: C.line, width: 0.75 } });
        s.addText(code, { x: x + 0.3, y: y + 0.06, w: 0.72, h: 0.62, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: g.color, valign: "middle", margin: 0 });
        s.addText(name, { x: x + 1.02, y: y + 0.06, w: gw - 1.24, h: 0.62, fontFace: BODY_FONT, fontSize: 10.3, color: C.text, valign: "middle", margin: 0 });
      });
      if (g.mods.length === 2) { // 短列补原则说明卡，避免下半空置
        const y = gy + 0.68 + g.mods.length * 0.86 + 0.08;
        s.addShape("roundRect", { x: x + 0.18, y, w: gw - 0.36, h: 4.1 - (y - gy) - 0.18, rectRadius: 0.07, fill: { color: C.white }, line: { color: C.line, width: 0.75 } });
        s.addText("职责原则", { x: x + 0.32, y: y + 0.1, w: gw - 0.6, h: 0.34, fontFace: BODY_FONT, fontSize: 10.5, bold: true, color: g.color, margin: 0 });
        s.addText(g.principle, { x: x + 0.32, y: y + 0.46, w: gw - 0.6, h: 3.3 - (y - gy), fontFace: BODY_FONT, fontSize: 9.8, color: C.mute, margin: 0 });
      } else if (g.mods.length === 3) { // 三模块列只剩窄条，放一行引言
        const y = gy + 0.68 + g.mods.length * 0.86 + 0.1;
        s.addText(g.principle, { x: x + 0.2, y, w: gw - 0.4, h: 4.1 - (y - gy) - 0.2, fontFace: BODY_FONT, fontSize: 9.5, italic: true, color: C.mute, valign: "top", margin: 0 });
      }
    });
    card(s, 0.62, 6.05, 12.08, 0.95, { fill: C.deep, noLine: true });
    iconCircle(s, "route", 0.88, 6.24, 0.56, C.forest);
    s.addText("架构主线：感知产出事实 → 模型产出可操作条件 → 业务作决定 → 执行侧复核并输出运动/工具命令；上级协调、下级完成能力任务",
      { x: 1.62, y: 6.05, w: 10.9, h: 0.95, fontFace: BODY_FONT, fontSize: 12.5, color: C.white, valign: "middle", margin: 0 });
    s.addNotes("12 模块按职责归四组呈现；来源 02_modules_and_boundaries.md。底部为主线原则（总纲 §4）。");
  }

  // ============ S5 完整作业闭环 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "01 WORKFLOW", "完整作业闭环：W01 → W08", 5);
    const steps = [
      ["W01", "任务准备", "区域/任务账本建立"],
      ["W02", "导航到作业点", "成熟 Nav2 承担导航"],
      ["W03", "稳定与工作交接", "移动-采摘互斥解除"],
      ["W04", "观察建模选果", "感知事实+操作条件"],
      ["W05", "执行单果周期", "接近/套入/剪切/撤离"],
      ["W06", "收集与单果结算", "承接/入箱独立证据"],
      ["W07", "资源处置离场", "换箱/卸载/补能请求"],
      ["W08", "推进与作业收尾", "跨点推进/异常恢复"],
    ];
    steps.forEach(([code, name, desc], i) => {
      const col = i % 4, row = Math.floor(i / 4);
      const x = 0.62 + col * 3.12, y = 1.75 + row * 1.95;
      card(s, x, y, 2.9, 1.75, { shadow: true });
      s.addShape("ellipse", { x: x + 0.18, y: y + 0.18, w: 0.66, h: 0.66, fill: { color: row ? C.peachDeep : C.forest }, line: { type: "none" } });
      s.addText(code, { x: x + 0.18, y: y + 0.18, w: 0.66, h: 0.66, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(name, { x: x + 0.95, y: y + 0.16, w: 1.9, h: 0.72, fontFace: TITLE_FONT, fontSize: 13.5, bold: true, color: C.text, valign: "middle", margin: 0 });
      s.addText(desc, { x: x + 0.2, y: y + 0.98, w: 2.55, h: 0.66, fontFace: BODY_FONT, fontSize: 10.5, color: C.mute, margin: 0 });
    });
    card(s, 0.62, 5.85, 12.08, 1.15, { fill: C.tint, noLine: true });
    const rules = [
      ["互斥", "移动与臂采摘互斥，稳定交接后才作业"],
      ["恢复", "结果未知进入受控恢复，不猜成功"],
      ["失效", "地图/工具变化使旧模型与计划失效"],
    ];
    rules.forEach(([k, v], i) => {
      const x = 0.88 + i * 4.0;
      s.addText(k, { x, y: 6.0, w: 0.82, h: 0.85, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: C.peachDeep, valign: "middle", margin: 0 });
      s.addText(v, { x: x + 0.72, y: 6.0, w: 3.1, h: 0.85, fontFace: BODY_FONT, fontSize: 10.5, color: C.text, valign: "middle", margin: 0 });
    });
    s.addNotes("W01-W08 来自 03_architecture_and_workflow.md；工作点内循环（W04-W06）、跨点循环（W07-W08）。");
  }

  // ============ S6 技术栈与工作区 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "02 TECH STACK", "现行技术栈与工作区布局", 6);
    // 左：硬件
    card(s, 0.62, 1.7, 5.55, 2.5, { shadow: true });
    iconCircle(s, "server", 0.88, 1.94, 0.58, C.forest);
    s.addText("硬件基线", { x: 1.6, y: 1.98, w: 3.4, h: 0.48, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "机械臂 AUBO E5 六轴 · 示教器上电/抱闸 · 驱动栈只读",
      "相机 Percipio RGB-D（腕载）+ PS800-E1 主机立体（默认前端 stereo）",
      "车载主控 Jetson Orin NX 16GB 整机栈 · RK3588 前端预留",
      "末端：三把剪切手 + 快换（STW）+ IMU 自适应档",
    ], 0.92, 2.52, 4.95, 1.6, { size: 11.5 });
    // 左下：软件栈
    card(s, 0.62, 4.4, 5.55, 2.55, { shadow: true });
    iconCircle(s, "chip", 0.88, 4.64, 0.58, C.peachDeep);
    s.addText("软件栈", { x: 1.6, y: 4.68, w: 3.4, h: 0.48, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "ROS 2 Jazzy · C++17 · Python 3.12 · numpy 1.26.4（cv_bridge ABI 锁定）",
      "MoveIt 2 + MTC + Pilz（PTP/LIN/CIRC）· 透传控制器",
      "Nav2 生命周期 + Gazebo Harmonic（gz_ros2_control）",
      "MCAP 会话记录 · FastDDS · industrial_ci（GitHub Actions）",
    ], 0.92, 5.22, 4.95, 1.65, { size: 11.5 });
    // 右：工作区布局
    card(s, 6.5, 1.7, 6.2, 5.25, { fill: C.tint, noLine: true });
    s.addText("工作区分组（2026-10 重组）", { x: 6.78, y: 1.92, w: 5.6, h: 0.45, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: C.forest, margin: 0 });
    const zones = [
      ["src/peach/ · 9 包", "现行采摘栈：interfaces / common / harvester（vision+supervisor+障碍） / arm / bringup / observability / vegetation / stereo / system_tests", C.forest],
      ["src/sim/ · 5 包", "仿真子工作空间：sim_worlds / sim_description / sim_bringup / sim_perception / sim_system_tests", C.peachDeep],
      ["src/ivg/ · 演示栈", "IVG 抓取演示三包 + Gazebo 验证环境（不进 harvest_system）", C.forest],
      ["src/ 顶层", "驱动九包（只读）+ serial_imu / imu_follow / percipio_camera / peach_web（8090）", C.peachDeep],
    ];
    zones.forEach(([t, d, color], i) => {
      const y = 2.44 + i * 1.04;
      s.addShape("roundRect", { x: 6.75, y, w: 5.7, h: 0.92, rectRadius: 0.08, fill: { color: C.white }, line: { color: C.line, width: 0.75 } });
      s.addText(t, { x: 6.95, y: y + 0.06, w: 5.3, h: 0.34, fontFace: TITLE_FONT, fontSize: 12, bold: true, color, margin: 0 });
      s.addText(d, { x: 6.95, y: y + 0.4, w: 5.35, h: 0.48, fontFace: BODY_FONT, fontSize: 9.5, color: C.mute, margin: 0 });
    });
    s.addText("包级活文档（doc/architecture.md + doc/io.md）为系统快照权威；peach2 已于 10-12 归档", { x: 6.78, y: 6.6, w: 5.7, h: 0.3, fontFace: BODY_FONT, fontSize: 9, italic: true, color: C.mute, margin: 0 });
    s.addNotes("硬件=固定座 E5+双相机前端；软件=Jazzy 全 apt；工作区四分组，驱动只读。");
  }

  // ============ S7 安全与权限 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "02 SAFETY", "安全与权限：三层分层 + 使能门", 7);
    const layers = [
      ["shield", "功能安全（柜与示教器）", "ISO 13850 急停 / ISO 10218 保护停止 / IEC 60204-1 停止类别", "硬件急停不经 ROS · 切断驱动电源 · 不远程上电/解锁", C.deep],
      ["heartbeat", "应用护栏（ROS 2 栈）", "ExecutionAuthority 使能门 · 命令超时与 bond 心跳 · 规划碰撞", "单一命令门：enables × clearance × robotReady × ¬cancel", C.forest],
      ["ban", "网络安全（边界）", "SROS2/DDS-Security 可选 · 网页仅表达意图", "不是功能安全 · 挡不住臂已在动 · 页面关闭≠任务结束", C.peachDeep],
    ];
    layers.forEach(([ic, t, d1, d2, color], i) => {
      const y = 1.72 + i * 1.5;
      card(s, 0.62, y, 7.3, 1.32, { shadow: true });
      iconCircle(s, ic, 0.88, y + 0.32, 0.66, color);
      s.addText(t, { x: 1.72, y: y + 0.1, w: 5.9, h: 0.4, fontFace: TITLE_FONT, fontSize: 14.5, bold: true, color, margin: 0 });
      s.addText(d1, { x: 1.72, y: y + 0.5, w: 6.0, h: 0.36, fontFace: BODY_FONT, fontSize: 11, color: C.text, margin: 0 });
      s.addText(d2, { x: 1.72, y: y + 0.86, w: 6.0, h: 0.36, fontFace: BODY_FONT, fontSize: 10.5, italic: true, color: C.mute, margin: 0 });
    });
    // 右：红线
    card(s, 8.2, 1.72, 4.5, 4.32, { fill: C.deep, noLine: true });
    iconCircle(s, "check", 8.46, 1.98, 0.56, C.moss);
    s.addText("MUST 红线", { x: 9.18, y: 2.02, w: 3.2, h: 0.48, fontFace: TITLE_FONT, fontSize: 15, bold: true, color: C.white, margin: 0 });
    bullets(s, [
      "未授权不得真机运动或 SetIO（操作员发起 real 指令 = 授权）",
      "硬件急停不经 ROS；软件停轨不称 e-stop",
      "驱动栈九包只读；bringup 不起 dashboard",
      "保护停止/急停后不 resume 原轨迹",
      "刀具与动臂同级危险：tool.enabled 默认关",
      "任何测试结束停干净并 pgrep 清进程",
    ], 8.5, 2.62, 3.95, 3.3, { color: "E7F0E2", size: 10.8, gap: 7 });
    card(s, 0.62, 6.28, 12.08, 0.8, { fill: C.tint, noLine: true });
    s.addText("使能链路：操作台 SetEnables（意图源 supervisor）→ /peach/batch/enables 广播 → 臂侧命令门强制；无广播时臂侧本地参数权威",
      { x: 0.9, y: 6.28, w: 11.6, h: 0.8, fontFace: BODY_FONT, fontSize: 11.5, color: C.text, valign: "middle", margin: 0 });
    s.addNotes("三层安全来自 AGENTS.md 第 2 章；使能链路为清洁重写轮现行口径。");
  }

  // ============ S8 两月演进 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "02 EVOLUTION", "近两月工程演进：205 笔提交", 8);
    // 左：柱状图
    card(s, 0.62, 1.7, 5.9, 4.0, { shadow: true });
    s.addText("周度提交数（2026 W34–W42）", { x: 0.9, y: 1.86, w: 5.2, h: 0.4, fontFace: BODY_FONT, fontSize: 12.5, bold: true, color: C.text, margin: 0 });
    s.addChart("bar", [
      { name: "提交数", labels: ["W34", "W35", "W36", "W37", "W38", "W39", "W40", "W41", "W42"], values: [12, 2, 9, 15, 71, 56, 13, 17, 10] },
    ], {
      x: 0.85, y: 2.3, w: 5.45, h: 3.25,
      barDir: "col", chartColors: [C.forest],
      showTitle: false, showValue: true, dataLabelPosition: "outEnd",
      dataLabelColor: C.mute, dataLabelFontSize: 9, dataLabelFontFace: BODY_FONT,
      catAxisLabelColor: C.mute, catAxisLabelFontSize: 10, valAxisHidden: true,
      valGridLine: { style: "none" }, catGridLine: { style: "none" },
      showLegend: false, valAxisMaxVal: 80,
    });
    // 右：时间线
    const tl = [
      ["09-20/21", "质量重构 W1–W16 + 感知优化轮：517 测 0 败", C.forest],
      ["09-22/24", "双末端战役 + E2E 方案 + 轨迹 v3/v4 + 真机首抓 9.4s", C.peachDeep],
      ["09-28", "三把剪切手 STEP→URDF · 感知矩阵 852 帧 GATE PASS", C.forest],
      ["09-29/30", "观测性四层全节点 + IVG 重构 + peach2 运动后端", C.peachDeep],
      ["10-08/10", "文档重组：包级活文档 + 工作区分组 + runs 清理", C.forest],
      ["10-12", "冻结相机 v1/v2 对比 + 深度清理（peach2 归档）", C.peachDeep],
      ["10-13/14", "仿真链 P1→P4b：统一机体→感知闭环→自主导航采摘", C.forest],
    ];
    tl.forEach(([d, t, color], i) => {
      const y = 1.72 + i * 0.6;
      s.addShape("roundRect", { x: 6.72, y, w: 0.98, h: 0.48, rectRadius: 0.07, fill: { color }, line: { type: "none" } });
      s.addText(d, { x: 6.72, y, w: 0.98, h: 0.48, fontFace: BODY_FONT, fontSize: 9.5, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(t, { x: 7.85, y: y - 0.04, w: 4.85, h: 0.56, fontFace: BODY_FONT, fontSize: 10.5, color: C.text, valign: "middle", margin: 0 });
      if (i < tl.length - 1) s.addShape("line", { x: 7.2, y: y + 0.48, w: 0, h: 0.12, line: { color: C.line, width: 1.5 } });
    });
    card(s, 0.62, 6.02, 12.08, 1.02, { fill: C.tint, noLine: true });
    const kv = [["205", "笔提交"], ["2558", "文件变更"], ["1.83M", "行入库（+）/ 6.6万（−）"], ["W38", "峰值周 71 笔（9-17~23）"]];
    kv.forEach(([n, l], i) => statBox(s, 0.95 + i * 3.0, 6.12, 2.8, n, l, i === 3 ? C.peachDeep : C.forest));
    s.addNotes("git log --since=2026-08-15：205 提交；峰值 W38/W39=双末端战役+重构期。时间线按提交主题归并。");
  }

  // ============ S9 感知系统 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "03 PERCEPTION", "感知系统：停走式节拍 · 事实产出", 9);
    // 左上 stereo 对比图
    card(s, 0.62, 1.7, 6.6, 3.4, { shadow: true });
    s.addText("立体深度对比（深度伪彩 + 点云）", { x: 0.88, y: 1.84, w: 5.9, h: 0.36, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.text, margin: 0 });
    picFit(s, "stereo_cmp.png", 0.82, 2.24, 6.2, 2.75);
    // 左下 masks
    card(s, 0.62, 5.3, 3.2, 1.72, { shadow: true });
    s.addText("相机流 RViz 窗口（Percipio 前端）", { x: 0.8, y: 5.4, w: 2.9, h: 0.32, fontFace: BODY_FONT, fontSize: 10, bold: true, color: C.text, margin: 0 });
    picFit(s, "percipio_view.png", 0.78, 5.74, 2.9, 1.2);
    card(s, 3.98, 5.3, 3.24, 1.72, { shadow: true });
    s.addText("YOLO 目标检测验证", { x: 4.16, y: 5.4, w: 2.9, h: 0.32, fontFace: BODY_FONT, fontSize: 10, bold: true, color: C.text, margin: 0 });
    picFit(s, "yolo_val.jpg", 4.14, 5.74, 2.92, 1.2);
    // 右：要点
    const items = [
      ["camera", "停走式节拍", "位拍照 → YOLO 检测（置信度 0.80+）→ SAM 实例掩膜 → 立体深度 → 多视点重建积分（精确 stamp TF）"],
      ["chart", "两月关键优化", "袋长 2D+3D 融合修正 −29% 系统性偏短 · 几何核两段式 preemptive RANSAC（512 粗排 / 16 决赛）· 掩膜门最近邻容差配对（stereo 前端解锁）"],
      ["database", "契约边界", "感知只发事实：不发运动命令、不选下一颗、不写账本（可写 perception_data 事件）"],
    ];
    items.forEach(([ic, t, d], i) => {
      const y = 1.7 + i * 1.82;
      card(s, 7.5, y, 5.2, 1.64, { shadow: true });
      iconCircle(s, ic, 7.74, y + 0.24, 0.6, i === 1 ? C.peachDeep : C.forest);
      s.addText(t, { x: 8.5, y: y + 0.12, w: 4.0, h: 0.4, fontFace: TITLE_FONT, fontSize: 13.5, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: 7.78, y: y + 0.56, w: 4.75, h: 1.0, fontFace: BODY_FONT, fontSize: 10.2, color: C.mute, margin: 0 });
    });
    s.addNotes("图为真实测试素材：stereo_cmp=peach_stereo 测试报告四窗格对比；masks=10-14 会话 712 帧掩膜序列中段帧；yolo_val=runs/pose/val。");
  }

  // ============ S10 运动执行 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "03 MOTION", "运动执行：接近轨迹四代演进", 10);
    // 右大图
    card(s, 6.35, 1.7, 6.35, 4.05, { shadow: true });
    s.addText("RViz 完整链（检测框 target_0 + 点云 + 规划轨迹）", { x: 6.6, y: 1.84, w: 5.9, h: 0.36, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.text, margin: 0 });
    picFit(s, "rviz_fullchain.png", 6.55, 2.24, 5.95, 3.4);
    // 左：四代
    const gens = [
      ["v1/v2", "staging 绕角 + 斜直线", "斜直线证伪：通过率 1/30", C.mute],
      ["v3", "PTP + 沿轴垂直 + 滚转梯子", "{0,±30,±60} 随机重启，typical 63% 下界", C.forest],
      ["v4", "世界垂直 LIN 入冠 0.05m", "解析成功率 57/80 = 71.3%，入冠零代价", C.forest],
      ["v4d", "Pilz sequence blend 0.02", "冠内双 LIN 速度连续 · 网格 18/20 · 走廊 13/13 零降级", C.peachDeep],
    ];
    gens.forEach(([g, t, d, color], i) => {
      const y = 1.7 + i * 1.02;
      card(s, 0.62, y, 5.55, 0.9, { shadow: true });
      s.addShape("roundRect", { x: 0.8, y: y + 0.17, w: 0.78, h: 0.56, rectRadius: 0.08, fill: { color: color === C.mute ? "9AA79B" : color }, line: { type: "none" } });
      s.addText(g, { x: 0.8, y: y + 0.17, w: 0.78, h: 0.56, fontFace: BODY_FONT, fontSize: 12, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(t, { x: 1.74, y: y + 0.08, w: 4.3, h: 0.4, fontFace: TITLE_FONT, fontSize: 12.5, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: 1.74, y: y + 0.47, w: 4.35, h: 0.36, fontFace: BODY_FONT, fontSize: 10, color: C.mute, margin: 0 });
    });
    // 底：执行要点
    card(s, 0.62, 5.92, 12.08, 1.1, { fill: C.tint, noLine: true });
    bullets(s, [
      "执行链：MoveIt/MTC 规划 → 透传控制器下发；boundedExecute 超时防线（TEM 关闭，决策 0029）",
      "命令门：enables × clearance × robotReady × ¬cancel；许可令牌 CONTACT/TOOL 分档余量（径向/轴向）",
    ], 0.95, 6.05, 11.5, 0.9, { size: 11.5, gap: 6 });
    s.addNotes("四代轨迹演进：v2 证伪 1/30 → v3 滚转梯子 63% → v4 入冠 71.3%（网格 18/20、走廊 13/13）→ v4d blend 连续。图为 10-12 冻结链 RViz 录屏帧。");
  }

  // ============ S11 末端工具 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "04 END EFFECTOR", "末端工具：三把 CAD 剪切手", 11);
    const tools = [
      ["shear_v1", "基准双刃剪切", "开口 +Y · 5.5 万/4.5k tri 双档网格", "档案 design_reference 传承"],
      ["bite_shear_v1", "咬合剪切（10-13 起默认）", "TCP (0, 30.24, 165.5) mm", "照片位 FULL 不可达定案后转 PREGRASP_ONLY"],
      ["adaptive_shear_v1", "IMU 自适应剪切", "TCP (0, 47, 168.66) mm · insert.max_travel 0.09", "imu_follow 插入模式跟随接触"],
    ];
    tools.forEach(([name, t, d1, d2], i) => {
      const x = 0.62 + i * 4.13;
      card(s, x, 1.72, 3.9, 2.6, { shadow: true });
      iconCircle(s, "cut", x + 0.24, 1.96, 0.62, i === 1 ? C.peachDeep : C.forest);
      s.addText(name, { x: x + 1.0, y: 1.94, w: 2.8, h: 0.4, fontFace: "Courier New", fontSize: 12.5, bold: true, color: C.text, margin: 0 });
      s.addText(t, { x: x + 1.0, y: 2.32, w: 2.85, h: 0.42, fontFace: BODY_FONT, fontSize: 10.5, bold: true, color: C.peachDeep, margin: 0 });
      bullets(s, [d1, d2], x + 0.26, 2.85, 3.45, 1.3, { size: 10, gap: 5, color: C.mute });
    });
    // 下：管线与证据链
    card(s, 0.62, 4.6, 5.9, 2.35, { shadow: true });
    iconCircle(s, "wrench", 0.88, 4.84, 0.58, C.forest);
    s.addText("STEP → URDF 管线", { x: 1.6, y: 4.88, w: 4.2, h: 0.45, fontFace: TITLE_FONT, fontSize: 14, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "FreeCAD 总装提取 + Blender 4.5 无头减面/叠对渲染",
      "法兰系 = CAD 全局系；三方数值叠对 + 渲染叠图双验收",
      "TCP 测量算术自洽核定（毫米级标定禁视觉转述）",
    ], 0.92, 5.4, 5.3, 1.4, { size: 11 });
    card(s, 6.75, 4.6, 5.95, 2.35, { shadow: true });
    iconCircle(s, "check", 7.0, 4.84, 0.58, C.peachDeep);
    s.addText("剪切证据链与许可", { x: 7.72, y: 4.88, w: 4.4, h: 0.45, fontFace: TITLE_FONT, fontSize: 14, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "ToolState 三轴 IDL + DI 事件门；VerifyCut 有界等沿保守收口（CUT_UNCONFIRMED 不猜成功）",
      "GraspDecision 许可令牌族只授权套入/剪切；PREGRASP_ONLY 不要求许可",
      "轴向预算 RSS 修正（v1 线性相加恒负已修）+ 分档余量门",
    ], 7.05, 5.4, 5.35, 1.4, { size: 11 });
    s.addNotes("938b5df/1aa50d0（09-28）：三剪切手档案 v1.2；TCP 数值为测量核定值。bite 为 10-13 操作员裁定默认档。");
  }

  // ============ S12 测试体系 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "04 TESTING", "测试体系：金字塔与验收门", 12);
    // 左：金字塔
    const pyr = [
      ["Lint（ament 全家桶）", 2.9, C.moss],
      ["纯核单测 pytest / gtest（零 ROS）", 3.7, C.forest],
      ["launch_testing 集成（隔离域）", 4.5, "3F7A40"],
      ["独立系统测包（peach_system_tests）", 5.3, C.deep],
      ["真机命名轮次（field_* · 授权+bag）", 6.1, "142B18"],
    ];
    pyr.forEach(([t, w, color], i) => {
      const y = 5.6 - i * 0.72, cx = 3.55, x = cx - w / 2;
      s.addShape("roundRect", { x, y, w, h: 0.62, rectRadius: 0.08, fill: { color }, line: { type: "none" } });
      s.addText(t, { x, y, w, h: 0.62, fontFace: BODY_FONT, fontSize: i < 2 ? 9.5 : 10.5, bold: i >= 2, color: C.white, align: "center", valign: "middle", margin: 0 });
    });
    s.addText("mock 不证明安全 · 来源分层记录不合并成功率 · 测完即停 pgrep 复核", { x: 0.62, y: 6.5, w: 6.3, h: 0.6, fontFace: BODY_FONT, fontSize: 10.5, italic: true, color: C.peachDeep, margin: 0 });
    // 右：数据卡
    const stats = [
      ["517 测 0 败", "W1–W16 质量轮终验（lint+纯核）"],
      ["E2E 64/64", "观测性四层轮全绿（09-29）"],
      ["launch_testing 5/5", "mock 整栈冒烟收编（隔离域）"],
      ["852 帧 GATE PASS", "感知验证矩阵首轮基线冻结（09-28）"],
      ["回放塔 51 用例", "stratified 49 + random 6·oracle 三路点重封"],
      ["campaign 矩阵 100%", "M1–M5 失败全归因三族边界（20 例网格）"],
    ];
    stats.forEach(([n, l], i) => {
      const col = i % 2, row = Math.floor(i / 2);
      const x = 7.15 + col * 2.85, y = 1.78 + row * 1.62;
      card(s, x, y, 2.7, 1.44, { shadow: true });
      s.addText(n, { x: x + 0.16, y: y + 0.14, w: 2.4, h: 0.52, fontFace: TITLE_FONT, fontSize: 14.5, bold: true, color: i % 2 ? C.peachDeep : C.forest, margin: 0 });
      s.addText(l, { x: x + 0.16, y: y + 0.66, w: 2.42, h: 0.7, fontFace: BODY_FONT, fontSize: 9.3, color: C.mute, margin: 0 });
    });
    s.addNotes("金字塔=AGENTS 第 10 章口径；数据卡全部来自提交与测试记录（517/E2E64/launch5/852帧/回放塔51/M1-M5 归因）。");
  }

  // ============ S13 真机验证 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "04 FIELD TESTS", "真机验证记录（授权轮次）", 13);
    const rounds = [
      ["S3 真相机首抓 · 09-24", "真实锁定 target_1（YOLO 0.84）→ PREGRASP 停驻 → ACK → COMPLETED", "周期 9.4s · 全程无 SetIO · 监测基建（45s 截图 / RViz 视频 / 内存守门）全程运行", C.peachDeep],
      ["S4 FULL 链 · 09-24", "接触前 ACK 停驻；袋口 |p|=1.079m 位于套入可达边界", "三轮 9.3s 到位等命令；驱动被外部取消未收口（记录在案）", C.forest],
      ["冻结相机 v1 稳定轮 · 10-12", "冻结 bag 上完整周期 ×4 复现", "28.4s/28.4s/28.4s/28.6s（approach 15.3 + retreat 12.5，σ≈0.1s）", C.forest],
      ["真机专项轮 · 10-14", "剪切链 / 外袋验证 / 稳定性闭环 ×5 / RViz 复跑 ×3", "会话 bag + ledger + geometry 全留痕；50G 预算执行 runs 87G→33MB", C.peachDeep],
    ];
    rounds.forEach(([t, d1, d2, color], i) => {
      const y = 1.72 + i * 1.28;
      card(s, 0.62, y, 12.1, 1.12, { shadow: true });
      s.addShape("roundRect", { x: 0.8, y: y + 0.14, w: 2.6, h: 0.84, rectRadius: 0.08, fill: { color }, line: { type: "none" } });
      s.addText(t, { x: 0.86, y: y + 0.14, w: 2.5, h: 0.84, fontFace: BODY_FONT, fontSize: 10.5, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(d1, { x: 3.6, y: y + 0.1, w: 9.0, h: 0.44, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.text, margin: 0 });
      s.addText(d2, { x: 3.6, y: y + 0.56, w: 9.0, h: 0.44, fontFace: BODY_FONT, fontSize: 10.5, color: C.mute, margin: 0 });
    });
    card(s, 0.62, 6.9, 12.1, 0.42, { fill: C.tint, noLine: true });
    s.addText("真机纪律：示教器上电 · 急停手能摸到 · 工作空间无人 · 先 mock 后 real · 采摘对错的最终权威在田间",
      { x: 0.9, y: 6.9, w: 11.6, h: 0.42, fontFace: BODY_FONT, fontSize: 10, color: C.text, valign: "middle", margin: 0 });
    s.addNotes("四轮真机记录：S3 首抓 9.4s（4b5936c）、S4 FULL、冻结 v1 28.4s×4（runs/frozen_v1_* ledger）、10-14 专项（runs/live_cam_shear 等）。");
  }

  // ============ S14 仿真体系 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "05 SIMULATION", "仿真体系：P1 → P4b 四步落地", 14);
    // 左图
    card(s, 0.62, 1.7, 5.3, 3.2, { shadow: true });
    s.addText("重建果园渲染（sim_worlds）", { x: 0.88, y: 1.84, w: 4.8, h: 0.36, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.text, margin: 0 });
    picFit(s, "sim_detail.png", 0.82, 2.22, 4.9, 2.6);
    card(s, 0.62, 5.05, 5.3, 1.95, { shadow: true });
    s.addText("光照矩阵（晨/午/阴/暮/逆光）", { x: 0.88, y: 5.16, w: 4.8, h: 0.32, fontFace: BODY_FONT, fontSize: 10, bold: true, color: C.text, margin: 0 });
    picFit(s, "sim_lighting_wide.jpg", 0.82, 5.5, 4.9, 1.4);
    // 右：P1-P4b
    const steps = [
      ["P1", "仿真子工作空间重组", "顶层 peach_sim 拆入 src/sim/ 五包：worlds / description / bringup / perception / system_tests"],
      ["P2", "统一机体", "gz_ros2_control 臂可控 + MoveIt + 可驾驶底盘一体（sim.launch.py）"],
      ["P3", "感知闭环", "peach 感知栈快照复制跑通仿真相机流（perception.launch.py）"],
      ["P4b", "自主导航采摘", "harvest.launch.py 一键（sim + perception + Nav2，field/程序化两预设）+ orchard_harvest CLI + generate_field 真值"],
    ];
    steps.forEach(([p, t, d], i) => {
      const y = 1.7 + i * 1.32;
      card(s, 6.2, y, 6.5, 1.16, { shadow: true });
      s.addShape("ellipse", { x: 6.4, y: y + 0.26, w: 0.64, h: 0.64, fill: { color: i === 3 ? C.peachDeep : C.forest }, line: { type: "none" } });
      s.addText(p, { x: 6.4, y: y + 0.26, w: 0.64, h: 0.64, fontFace: BODY_FONT, fontSize: 10.5, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(t, { x: 7.2, y: y + 0.1, w: 5.3, h: 0.4, fontFace: TITLE_FONT, fontSize: 13.5, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: 7.2, y: y + 0.5, w: 5.4, h: 0.6, fontFace: BODY_FONT, fontSize: 9.8, color: C.mute, margin: 0 });
    });
    card(s, 6.2, 6.98, 6.5, 0.4, { fill: C.tint, noLine: true });
    s.addText("同一管线原则：只换硬件插件与 use_sim_time，不写第二套关节话题",
      { x: 6.4, y: 6.98, w: 6.2, h: 0.4, fontFace: BODY_FONT, fontSize: 10, color: C.text, valign: "middle", margin: 0 });
    s.addNotes("P1-P4b=3eedfb2→98aa8ce（10-13/14）。图为重建果园真实渲染与光照检查板（sim_worlds reconstruction output）。");
  }

  // ============ S15 观测性与 Web ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "05 OBSERVABILITY", "观测性与 Web 工作台", 15);
    // 右图 web
    card(s, 6.9, 1.7, 5.8, 3.9, { shadow: true });
    s.addText("8090 调试台（纯调试客户端 · 绕不过命令门）", { x: 7.14, y: 1.84, w: 5.3, h: 0.36, fontFace: BODY_FONT, fontSize: 11.5, bold: true, color: C.text, margin: 0 });
    picFit(s, "web_console.png", 7.1, 2.26, 5.4, 3.2);
    card(s, 6.9, 5.78, 5.8, 1.28, { shadow: true });
    s.addText("peach_web 3D 面板 · 8090 日志面板（10-12/13 入库）", { x: 7.14, y: 5.92, w: 5.3, h: 0.4, fontFace: BODY_FONT, fontSize: 11, bold: true, color: C.text, margin: 0 });
    s.addText("Web 展示与表达意图；业务服务与执行侧管理权威，页面关闭 ≠ 任务结束", { x: 7.14, y: 6.32, w: 5.3, h: 0.6, fontFace: BODY_FONT, fontSize: 10.5, color: C.mute, margin: 0 });
    // 左：四层
    const feats = [
      ["chart", "/diagnostics 全节点聚合", "diagnostic_updater 全节点覆盖（peach_arm 9 任务、感知/重建/调度等）；observability 聚合进 8090"],
      ["database", "会话 bag MCAP", "随 observability 启停；runs/session_* 留 selfcheck、startup 工件；runs 87G→33MB 预算化治理"],
      ["book", "账本与证据", "ledger.json（终局/outcome/阶段时长）+ geometry.jsonl + perception_data 事件；request_id 不复用"],
      ["bolt", "自检与闩锁", "selfcheck 闩锁 + startup.json 工件 + blockers；失败三通道（日志/诊断/账本 failure_code）"],
    ];
    feats.forEach(([ic, t, d], i) => {
      const y = 1.7 + i * 1.36;
      card(s, 0.62, y, 6.05, 1.2, { shadow: true });
      iconCircle(s, ic, 0.86, y + 0.26, 0.6, i % 2 ? C.peachDeep : C.forest);
      s.addText(t, { x: 1.62, y: y + 0.1, w: 4.8, h: 0.4, fontFace: TITLE_FONT, fontSize: 13, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: 1.62, y: y + 0.5, w: 4.85, h: 0.62, fontFace: BODY_FONT, fontSize: 9.8, color: C.mute, margin: 0 });
    });
    s.addNotes("观测性四层=决策 0034-0037（09-29）落地；web 截图为 .playwright-mcp 实拍。");
  }

  // ============ S16 优化前后对比 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "06 BEFORE / AFTER", "优化前后对比：两月关键指标", 16);
    const header = [
      { text: "维度", options: { bold: true, color: C.white, fill: { color: C.deep }, align: "left" } },
      { text: "优化前", options: { bold: true, color: C.white, fill: { color: C.deep }, align: "left" } },
      { text: "优化后", options: { bold: true, color: C.white, fill: { color: C.forest }, align: "left" } },
    ];
    const rows = [
      ["接近轨迹通过率", "v2 斜直线证伪：1/30", "v4 世界垂直入冠 71.3%（网格 18/20）"],
      ["轨迹连续性", "PTP 分段停点", "Pilz sequence blend 冠内速度连续（v4d）"],
      ["袋长估计", "系统性偏短 −29%", "2D+3D 融合 + 检测框硬限幅修正"],
      ["几何核耗时", "全量 RANSAC 打分", "两段式 preemptive：512 粗排 / 16 决赛"],
      ["轴向安全预算", "线性相加恒负 → cut_ok 永 false", "RSS 修正 + CONTACT/TOOL 分档余量门"],
      ["测试规模", "少量纯核 pytest", "517 测 0 败 + E2E 64/64 + launch_testing 5/5"],
      ["感知基线", "无冻结基线", "852 帧矩阵首轮 GATE PASS 冻结"],
      ["诊断覆盖", "仅 3 包接入", "全节点 /diagnostics 聚合进 8090"],
    ];
    const tbl = [header, ...rows.map((r, ri) => r.map((c, ci) => ({
      text: c,
      options: {
        align: "left", bold: ci === 0,
        color: ci === 1 ? C.mute : ci === 2 ? C.text : C.text,
        fill: { color: ri % 2 ? C.tint : "FFFFFF" },
      },
    })))];
    s.addTable(tbl, {
      x: 0.62, y: 1.72, w: 8.3, colW: [2.0, 3.0, 3.3],
      fontFace: BODY_FONT, fontSize: 10.3, rowH: 0.52,
      border: { pt: 0.5, color: C.line }, margin: 0.07, valign: "middle",
    });
    // 右：视频导看卡
    card(s, 9.2, 1.72, 3.5, 3.1, { shadow: true });
    s.addText("对比视频导看", { x: 9.42, y: 1.9, w: 3.0, h: 0.4, fontFace: TITLE_FONT, fontSize: 14, bold: true, color: C.text, margin: 0 });
    bullets(s, [
      "完整链 RViz（v1 栈 28.4s 周期实录）",
      "立体深度四窗格前后对比",
      "相机流 · 掩膜序列 · 数据字幕卡",
    ], 9.44, 2.36, 3.05, 1.6, { size: 10.5, gap: 6 });
    s.addText("交付物：optimization_before_after.mp4", { x: 9.42, y: 4.28, w: 3.1, h: 0.4, fontFace: BODY_FONT, fontSize: 9.5, italic: true, color: C.peachDeep, margin: 0 });
    card(s, 9.2, 5.05, 3.5, 1.95, { fill: C.deep, noLine: true });
    s.addText("单果周期（冻结 v1）", { x: 9.44, y: 5.25, w: 3.0, h: 0.4, fontFace: BODY_FONT, fontSize: 11, bold: true, color: C.moss, margin: 0 });
    s.addText("28.4s", { x: 9.44, y: 5.6, w: 3.0, h: 0.85, fontFace: TITLE_FONT, fontSize: 40, bold: true, color: C.white, margin: 0 });
    s.addText("×4 复现 σ≈0.1s · approach 15.3s + retreat 12.5s", { x: 9.44, y: 6.48, w: 3.05, h: 0.5, fontFace: BODY_FONT, fontSize: 9.5, color: "BFD3B8", margin: 0 });
    s.addNotes("对比表全部来自提交记录与 runs 账本；28.4s 数据=frozen_v1 四轮 ledger 均值。视频同目录交付。");
  }

  // ============ S17 路线图 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.bg };
    titleBar(s, "06 ROADMAP", "建设路线：G0 → G4 分期验收", 17);
    const gs = [
      ["G0", "完整框架与模拟闭环", "12 模块 + 任务层级 + 账本页面全走通（模拟不冒充物理验收）", "仿真侧已贯通（P4b 一键）", C.forest],
      ["G1", "移动平台与工作点", "成熟定位/导航接入 · 稳定交接 · 移动-采摘互斥有效", "待建（产品底盘选型）", C.mute],
      ["G2", "固定工位完整采摘", "单果采摘/收集独立证据 · 缺反馈不结算成功", "固定座 v1 真机基础具备", C.forest],
      ["G3", "导航-采摘-物流集成", "跨点不重复不丢终局 · 资源约束可中断重编排", "待建", C.mute],
      ["G4", "果园验证与优化", "光照/遮挡/地形分层统计 · 数据驱动优化", "待建（指标分母先定义）", C.peachDeep],
    ];
    gs.forEach(([g, t, d, status, color], i) => {
      const x = 0.62 + i * 2.47;
      card(s, x, 1.75, 2.32, 3.6, { shadow: true });
      s.addShape("roundRect", { x: x + 0.16, y: 1.95, w: 0.78, h: 0.5, rectRadius: 0.08, fill: { color: color === C.mute ? "9AA79B" : color }, line: { type: "none" } });
      s.addText(g, { x: x + 0.16, y: 1.95, w: 0.78, h: 0.5, fontFace: BODY_FONT, fontSize: 13, bold: true, color: C.white, align: "center", valign: "middle", margin: 0 });
      s.addText(t, { x: x + 0.16, y: 2.56, w: 2.0, h: 0.75, fontFace: TITLE_FONT, fontSize: 12, bold: true, color: C.text, margin: 0 });
      s.addText(d, { x: x + 0.16, y: 3.3, w: 2.02, h: 1.15, fontFace: BODY_FONT, fontSize: 9.3, color: C.mute, margin: 0 });
      s.addShape("roundRect", { x: x + 0.14, y: 4.6, w: 2.04, h: 0.6, rectRadius: 0.07, fill: { color: color === C.mute ? "EDF1EA" : C.tint }, line: { type: "none" } });
      s.addText(status, { x: x + 0.2, y: 4.6, w: 1.94, h: 0.6, fontFace: BODY_FONT, fontSize: 9, bold: color !== C.mute, color: color === C.mute ? C.mute : C.forest, align: "center", valign: "middle", margin: 0 });
      if (i < 4) s.addText("→", { x: x + 2.28, y: 3.3, w: 0.24, h: 0.5, fontFace: BODY_FONT, fontSize: 15, color: C.mute, align: "center", margin: 0 });
    });
    card(s, 0.62, 5.65, 12.08, 1.35, { fill: C.tint, noLine: true });
    s.addText("贯穿验收维度", { x: 0.9, y: 5.8, w: 3.0, h: 0.4, fontFace: TITLE_FONT, fontSize: 12.5, bold: true, color: C.forest, margin: 0 });
    bullets(s, [
      "完整闭环：正常 / 取消 / 资源不足 / 导航失败 / 剪切未知 / 传感失联 / 人工恢复全覆盖",
      "来源分层：模拟、回放、真实相机 mock 臂、真机分别记录，不合并成功率；运维：可部署可回读可取消可诊断可收尾",
    ], 0.92, 6.22, 11.5, 0.72, { size: 10.5, gap: 5 });
    s.addNotes("G0-G4=05_delivery_roadmap；状态标注为 06_current_progress_mapping 的进度线索（不宣称百分比）。");
  }

  // ============ S18 尾页 ============
  {
    const s = pres.addSlide();
    s.background = { color: C.deep };
    s.addText("PEACH · 2026-10", { x: 0.75, y: 1.0, w: 6.0, h: 0.4, fontFace: BODY_FONT, fontSize: 13, color: C.moss, bold: true, charSpacing: 3, margin: 0 });
    s.addText("设计定型 · 工程闭环 · 田间待验", { x: 0.7, y: 1.5, w: 11.8, h: 1.0, fontFace: TITLE_FONT, fontSize: 36, bold: true, color: C.white, margin: 0 });
    const sums = [
      ["book", "设计", "12 模块完整框架 + G0-G4 路线（peach_autonomous_harvest_design）"],
      ["branch", "工程", "两月 205 提交：质量重构 → 双末端战役 → 真机首抓 → 仿真自主采摘"],
      ["flag", "验证", "真机首抓 9.4s · 冻结链 28.4s×4 · 852 帧感知基线 · 517 测 0 败"],
    ];
    sums.forEach(([ic, t, d], i) => {
      const x = 0.75 + i * 4.1;
      s.addShape("roundRect", { x, y: 2.9, w: 3.8, h: 1.7, rectRadius: 0.1, fill: { color: "27502F" }, line: { type: "none" } });
      iconCircle(s, ic, x + 0.24, 3.12, 0.56, C.moss);
      s.addText(t, { x: x + 0.95, y: 3.1, w: 2.6, h: 0.5, fontFace: TITLE_FONT, fontSize: 16, bold: true, color: C.white, margin: 0 });
      s.addText(d, { x: x + 0.28, y: 3.72, w: 3.3, h: 0.8, fontFace: BODY_FONT, fontSize: 10.5, color: "D9E7D2", margin: 0 });
    });
    s.addText("交付物：项目设计 PPT · 优化前后对比视频（同目录）· 素材与账本索引（runs/ + git）",
      { x: 0.75, y: 5.1, w: 11.5, h: 0.5, fontFace: BODY_FONT, fontSize: 13, color: C.peach, margin: 0 });
    s.addText("安全声明：本报告数据全部来自 git 提交记录与 runs/ 测试账本；未包含任何未授权真机运动。",
      { x: 0.75, y: 6.5, w: 11.5, h: 0.45, fontFace: BODY_FONT, fontSize: 10.5, color: "8FA98A", margin: 0 });
    s.addNotes("尾页摘要 + 安全声明。");
  }

  await pres.writeFile({ fileName: path.join(ROOT, "桃采摘机器人项目设计_2026-10.pptx") });
  console.log("PPTX written");
}

main().catch((e) => { console.error(e); process.exit(1); });
