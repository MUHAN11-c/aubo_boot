const path = require("path");
const pptxgen = require("pptxgenjs");

const OUT = path.resolve(__dirname, "peach_force_harvest_20261023.pptx");
const A = (f) => path.resolve(__dirname, "assets", f);

const C = {
  ink: "1E2430",
  ink2: "2A3240",
  muted: "6B7280",
  dim: "AEB6C2",
  line: "E6E1D8",
  paper: "FFFFFF",
  cream: "FBF8F3",
  peach: "E76F51",
  coral: "F4A261",
  teal: "2A9D8F",
  sand: "E9C46A",
  blue: "457B9D",
  white: "FFFFFF",
};

const FONT = "Microsoft YaHei";
const NUM = "Arial";

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Peach Harvest";
pptx.company = "aubo_e5_jazzy_ws";
pptx.subject = "五点力 / 真机抓取 / CoP-剪切位置";
pptx.title = "桃采摘真机与五点力数据汇报 2026-10-23";
pptx.lang = "zh-CN";
pptx.theme = { headFontFace: FONT, bodyFontFace: FONT, lang: "zh-CN" };

const shadow = () => ({
  type: "outer",
  color: "000000",
  blur: 10,
  angle: 90,
  distance: 2,
  opacity: 0.08,
});

function text(slide, str, x, y, w, h, opts = {}) {
  slide.addText(str, {
    x, y, w, h,
    fontFace: FONT,
    fontSize: 12,
    color: C.ink,
    margin: 0,
    valign: "top",
    breakLine: false,
    ...opts,
  });
}

function eyebrow(slide, str, dark = false) {
  text(slide, str, 0.58, 0.38, 11.8, 0.28, {
    fontSize: 9.5,
    bold: true,
    color: dark ? C.coral : C.peach,
    charSpacing: 1.1,
  });
}

function footer(slide, str, n, dark = false) {
  text(slide, str, 0.58, 7.02, 10.4, 0.24, {
    fontSize: 7.6,
    color: dark ? "8E97A6" : "8A93A0",
  });
  text(slide, `${String(n).padStart(2, "0")} / 04`, 12.05, 6.98, 0.7, 0.24, {
    fontSize: 8.5,
    color: dark ? "8E97A6" : "8A93A0",
    align: "right",
  });
}

function card(slide, x, y, w, h, fill, dark = false) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.055,
    fill: { color: fill },
    line: dark ? { color: fill, width: 0 } : { color: C.line, width: 0.75 },
    shadow: dark ? undefined : shadow(),
  });
}

/* ------------------------------------------------------------------ P1 */
{
  const s = pptx.addSlide();
  s.background = { color: C.ink };
  eyebrow(s, "PEACH HARVEST  ·  真机测试  ·  2026-10-22", true);

  text(s, "真机 46 颗：41 颗套入到位", 0.55, 0.78, 6.55, 0.72, {
    fontSize: 27,
    bold: true,
    color: C.white,
    lineSpacingMultiple: 1.04,
  });

  s.addText(
    [
      { text: "89.1", options: { fontSize: 104, bold: true, color: C.peach, fontFace: NUM } },
      { text: "%", options: { fontSize: 46, bold: true, color: C.peach, fontFace: NUM } },
    ],
    { x: 0.5, y: 1.72, w: 5.7, h: 1.55, margin: 0, valign: "middle" }
  );
  text(s, "视觉引导抓取成功率", 0.58, 3.32, 5.9, 0.4, {
    fontSize: 18,
    bold: true,
    color: C.white,
  });
  text(s, "41 / 46  ·  5 次失败（4 接触异常 · 1 不可达）", 0.58, 3.78, 6.2, 0.32, {
    fontSize: 12.5,
    color: C.dim,
  });

  const chips = [
    ["18.5 s", "单颗周期中位"],
    ["9.8 / 8.0 s", "接近 / 撤退中位"],
    ["0 / 46", "采摘确认（tool 未开）"],
  ];
  chips.forEach((c, i) => {
    const x = 0.58 + i * 2.02;
    text(s, c[0], x, 4.62, 1.9, 0.42, {
      fontSize: 19,
      bold: true,
      color: i === 2 ? C.peach : C.white,
      fontFace: NUM,
    });
    text(s, c[1], x, 5.02, 1.9, 0.26, { fontSize: 8.8, color: C.dim });
  });
  text(s, "target_0 14/14  ·  target_1 12/12  ·  target_2 8/10  ·  target_3 7/10", 0.58, 5.62, 6.2, 0.3, {
    fontSize: 10.5,
    color: C.white,
  });
  text(s, "46/46 次均有有效视觉目标；5 次失败全部在 target_2/3（接触 4 · 可达 1）。", 0.58, 5.98, 6.2, 0.5, {
    fontSize: 9.2,
    color: C.dim,
    lineSpacingMultiple: 1.2,
  });

  card(s, 7.12, 0.98, 5.62, 5.62, C.white);
  text(s, "46 次抓取的结果分布", 7.42, 1.22, 4.9, 0.28, {
    fontSize: 9.5,
    bold: true,
    color: C.muted,
    charSpacing: 0.5,
  });

  s.addChart(
    pptx.ChartType.doughnut,
    [{
      name: "结果",
      labels: ["成功 41", "接触异常 4", "不可达 1"],
      values: [41, 4, 1],
    }],
    {
      x: 7.28, y: 1.42, w: 2.95, h: 2.95,
      chartColors: [C.teal, C.peach, C.sand],
      holeSize: 63,
      showLegend: false,
      showValue: false,
      dataBorder: { pt: 2.2, color: C.white },
      chartArea: { fill: { color: C.white }, border: { pt: 0, color: C.white } },
    }
  );
  s.addText(
    [
      { text: "41 / 46", options: { fontSize: 18, bold: true, color: C.ink, fontFace: NUM, breakLine: true } },
      { text: "成功", options: { fontSize: 8, color: C.muted } },
    ],
    { x: 8.08, y: 2.48, w: 1.55, h: 0.85, margin: 0, align: "center", valign: "middle" }
  );

  const legend = [
    [C.teal, "成功", "41", "89.1%"],
    [C.peach, "接触异常", "4", "8.7%"],
    [C.sand, "不可达", "1", "2.2%"],
  ];
  legend.forEach((l, i) => {
    const y = 1.92 + i * 0.43;
    s.addShape(pptx.ShapeType.ellipse, {
      x: 10.5, y: y + 0.05, w: 0.12, h: 0.12,
      fill: { color: l[0] }, line: { color: l[0], width: 0 },
    });
    text(s, l[1], 10.7, y - 0.02, 0.95, 0.26, { fontSize: 9.6, color: C.ink });
    text(s, l[2], 11.62, y - 0.02, 0.35, 0.26, { fontSize: 9.6, bold: true, color: C.ink, align: "right" });
    text(s, l[3], 12.02, y - 0.02, 0.5, 0.26, { fontSize: 8.4, color: C.muted, align: "right" });
  });
  text(s, "level 6 = 套入 + 撤退；\ntool 未开，因此没有 level 7。", 10.5, 3.35, 1.95, 0.62, {
    fontSize: 8.2,
    color: C.muted,
    lineSpacingMultiple: 1.16,
  });

  text(s, "分目标成功率 / %", 7.42, 4.42, 3.0, 0.24, {
    fontSize: 9.5,
    bold: true,
    color: C.muted,
  });
  s.addChart(
    pptx.ChartType.bar,
    [{
      name: "成功率",
      labels: ["target_0", "target_1", "target_2", "target_3"],
      values: [100, 100, 80, 70],
    }],
    {
      x: 7.32, y: 4.62, w: 5.15, h: 1.78,
      barDir: "col",
      chartColors: [C.teal, C.teal, C.peach, C.peach],
      showValue: true,
      dataLabelColor: C.ink,
      dataLabelFontSize: 9,
      dataLabelPosition: "outEnd",
      valAxisHidden: true,
      valAxisMaxVal: 100,
      valAxisMinVal: 0,
      catAxisLabelColor: C.ink,
      catAxisLabelFontSize: 7.8,
      catAxisLineShow: false,
      catAxisMajorTickMark: "none",
      valAxisLineShow: false,
      catGridLine: { style: "none" },
      valGridLine: { style: "none" },
      showLegend: false,
      barGapWidthPct: 48,
      chartArea: { fill: { color: C.white }, border: { pt: 0, color: C.white } },
    }
  );

  footer(s, "口径：视觉引导套入 + 撤退，completion_level ≥ 6；2026-10-22 16:37–17:06，session_20261022_163539；全天 55 周期 49 次套入到位（89.1%）。", 1, true);

  s.addNotes(
    "真机 2026-10-22 末段连续作业：session_20261022_163539，46 个抓取周期，41 个达到 completion_level 6（套入+撤退），成功率 89.1%。\n" +
    "5 次失败：4 次 skipped_quality（接触阶段取消或撤离未确认，target_2/3），1 次 skipped_unreachable（入冠滚转档套入直线未走满，target_3）。\n" +
    "tool.enabled=false，因此 completion_level 7 / harvest_confirmed = 0；这是配置，不是抓取失败。全天 55 周期 49 次套入到位，同样 89.1%；严格按 outcome=0 且 level≥6 为 47/55（85.5%）。\n" +
    "周期中位 18.5 s，接近 9.8 s，撤退 8.0 s。"
  );
}

/* ------------------------------------------------------------------ P2 */
{
  const s = pptx.addSlide();
  s.background = { color: C.paper };
  eyebrow(s, "力测试  ·  2026-10-21 台架  /  2026-10-22 真机");
  text(s, "五点力链路可信，但五点同时只出现过一次", 0.55, 0.72, 12.2, 0.62, {
    fontSize: 25,
    bold: true,
    color: C.ink,
  });
  text(s, "台架只验证到单通道 / 顺序加载；真机 target_0 留下 4.26 s 五点同步记录，才第一次具备 CoP 分析条件。", 0.57, 1.28, 12.0, 0.3, {
    fontSize: 10.2,
    color: C.muted,
  });

  s.addImage({ path: A("p2_manual_20s.png"), x: 0.45, y: 1.62, w: 6.05, h: 2.74 });
  s.addImage({ path: A("p2_real5_episode.png"), x: 6.78, y: 1.62, w: 6.05, h: 2.74 });

  text(s, "台架 20 s：5 路逐个满行程；五点同时 0 帧", 0.48, 4.42, 6.0, 0.26, {
    fontSize: 8.8,
    color: C.muted,
  });
  text(s, "真机 target_0：172 帧五点同时；峰值 5.24 N 与插入结束对齐", 6.8, 4.42, 6.1, 0.26, {
    fontSize: 8.8,
    color: C.muted,
  });

  const stats = [
    ["100.6 Hz", "采样率 · 997 帧 / 9.9 s"],
    ["65.8 g/mm", "力-行程斜率 · r = 0.9998"],
    ["< 1 g", "力残差（AD 复算）"],
    ["0 帧", "台架五点同时（3 组测试）"],
  ];
  stats.forEach((v, i) => {
    const x = 0.45 + i * 3.12;
    card(s, x, 4.82, 2.95, 1.12, C.cream);
    text(s, v[0], x + 0.18, 4.98, 2.6, 0.42, {
      fontSize: 22,
      bold: true,
      color: i === 3 ? C.peach : C.ink,
      fontFace: NUM,
    });
    text(s, v[1], x + 0.18, 5.46, 2.64, 0.28, { fontSize: 8.4, color: C.muted });
  });

  text(s, "通道 1 右上 / 2 右下 / 3 左下 / 5 上 / 7 左上（果实侧）。10 s 单点 3.619 N；20 s 四路顺序 3.628 N SAT；手动五点逐点 3.628 N / 6.0 mm；真机五点合计 5.237 N、CoP 8.19 mm。满行程 AD 1665–2250（极差 585）。", 0.55, 6.18, 12.3, 0.55, {
    fontSize: 8.9,
    color: C.muted,
    lineSpacingMultiple: 1.2,
  });

  footer(s, "数据源：runs/force_10s_20261021_095244 · force_20s_20261021_102812 · force_20s_manual · session_20261022_150712/force/episode_001.csv；370 g 饱和为固件钳位。", 2);

  s.addNotes(
    "10 s 测试：997 帧 / 9.9 s / 100.6 Hz，仅压通道 5，峰值 3.619 N、行程 5.6 mm，力-行程 65.8 g/mm（r=0.9998）；两次 6 mm 哨兵共 143 帧。\n" +
    "20 s 测试：1997 帧 / 20 s / 99.8 Hz，四路顺序按压，ch1 3.628 N 饱和 52 帧，ch2 2.432 N、ch5 1.226 N、ch7 3.060 N，ch3 未压；最多 2 路同时 >10 g。\n" +
    "手动 20 s：五点都到 3.628 N / 6.0 mm / SAT，但全 5 路同时 >10 g 为 0 帧；最多 2 路 >10 g（3 路仅 >1 g）。满行程 AD：2250/1665/1750/1967/1931。\n" +
    "真机：2026-10-22 15:09:34–38，target_0，426 帧 / 4.26 s，172 帧五点同时 >0.1 N，合计峰值 5.237 N，峰值帧 CoP 8.19 mm。\n" +
    "注：台架 mm 是力/66 的力等效行程，不是独立位移；绝对力仍未砝码标定。"
  );
}

/* ------------------------------------------------------------------ P3 */
{
  const s = pptx.addSlide();
  s.background = { color: C.paper };
  eyebrow(s, "真实五点同步  ·  target_0  ·  2026-10-22 15:09:34–38  ·  426 帧 / 4.26 s");
  text(s, "CoP 纠偏不是免费的：5° 旋转把刀口横移 3.24 mm", 0.55, 0.72, 12.2, 0.55, {
    fontSize: 24,
    bold: true,
    color: C.ink,
  });
  text(s, "315 帧有效 CoP（≥3 点 >0.1 N 且合力 >1 N），始终在 TCP −X 侧；左两点合计 2.95 N : 右两点 0.30 N（9.8 : 1）。", 0.57, 1.25, 12.1, 0.3, {
    fontSize: 10.2,
    color: C.muted,
  });

  s.addImage({ path: A("p3_sideview.png"), x: 0.38, y: 1.45, w: 3.45, h: 4.12 });
  s.addImage({ path: A("p3_padplane.png"), x: 3.95, y: 1.45, w: 3.2, h: 3.2 });

  text(s, "绕 TCP 5°：膜片 6.32 / 刀口 3.24 mm", 0.38, 5.62, 3.5, 0.3, {
    fontSize: 8.4,
    color: C.muted,
  });
  text(s, "压头盘面 · 圆内为平均力 / N · 五点为 CAD 位置", 3.95, 4.7, 3.25, 0.4, {
    fontSize: 8.1,
    color: C.muted,
    lineSpacingMultiple: 1.15,
  });

  text(s, "3.24", 7.35, 1.42, 2.2, 0.72, {
    fontSize: 46,
    bold: true,
    color: C.peach,
    fontFace: NUM,
  });
  text(s, "mm", 8.92, 1.66, 0.6, 0.35, {
    fontSize: 15,
    bold: true,
    color: C.peach,
    fontFace: NUM,
  });
  text(s, "5° 纠偏的刀口横移", 7.38, 2.12, 5.0, 0.3, {
    fontSize: 12.5,
    bold: true,
    color: C.ink,
  });
  text(s, "= 40% 刀口捕获窗口（8 mm）  ·  108% 刀面标定误差（3 mm）", 7.38, 2.44, 5.1, 0.28, {
    fontSize: 9.2,
    color: C.peach,
  });

  const rows = [
    [
      { text: "工况", options: { bold: true, color: C.white, fill: { color: C.ink }, align: "left" } },
      { text: "CoP / mm", options: { bold: true, color: C.white, fill: { color: C.ink } } },
      { text: "需转 θ", options: { bold: true, color: C.white, fill: { color: C.ink } } },
      { text: "5° 后残余", options: { bold: true, color: C.white, fill: { color: C.ink } } },
      { text: "刀口移 / mm", options: { bold: true, color: C.white, fill: { color: C.ink } } },
    ],
    ["峰值帧 5.24 N", "8.19", "6.5°", "1.87", "3.24"],
    ["P50（315 帧）", "8.61", "6.8°", "2.29", "3.24"],
    ["P90", "12.01", "9.4°", "5.68", "3.24"],
    ["5° 上限", "6.32", "5.0°", "0", "3.24"],
  ];
  const body = rows.slice(1).map((r, i) => r.map((v, j) => ({
    text: String(v),
    options: {
      color: C.ink,
      fill: { color: i % 2 === 0 ? C.white : C.cream },
      align: j === 0 ? "left" : "center",
    },
  })));
  s.addTable([rows[0], ...body], {
    x: 7.35, y: 2.9, w: 5.4, h: 1.62,
    colW: [1.48, 0.88, 0.78, 1.13, 1.13],
    rowH: 0.3,
    fontFace: FONT,
    fontSize: 8.6,
    valign: "middle",
    margin: 0.03,
    border: { type: "solid", color: C.line, pt: 0.5 },
  });

  card(s, 7.35, 4.72, 5.4, 1.26, C.cream);
  text(s, "纠到 2 mm 的代价", 7.53, 4.86, 2.4, 0.24, {
    fontSize: 9.2,
    bold: true,
    color: C.peach,
  });
  text(s, "P50 需 5.2° / 3.38 mm；P90 需 7.9° / 5.12 mm，已超 5° 上限。CoP > 8 mm 时，5° 只能把残余压到 1.9–2.3 mm。", 7.53, 5.12, 5.05, 0.72, {
    fontSize: 8.9,
    color: C.ink,
    lineSpacingMultiple: 1.18,
  });

  text(s, "结论：CoP 8–12 mm 时，5° 纠偏只能部分回中，却固定把刀口推 3.24 mm（−X，CoP 同侧）—— 先限幅，再闭环。", 0.42, 6.3, 12.2, 0.34, {
    fontSize: 10.2,
    bold: true,
    color: C.ink,
  });

  footer(s, "几何：膜片平面 TCP z = −72.29 mm；bite_shear_v1 刀口 L_blade = 37 mm；侧视真实比例。CoP → θ 为一阶杠杆模型；刀口位移为刚体几何精确值。", 3);

  s.addNotes(
    "同步记录：runs/session_20261022_150712/force/episode_001.csv，target_0 真机周期，426 帧 / 4.26 s；峰值 5.237 N 出现在 2.405 s，紧邻插入结束/撤退开始（约 0.2 s）。\n" +
    "有效 CoP 帧：≥3 点 >0.1 N 且合力 >1 N，共 315 帧。CoP 均值 (−9.19, +0.59) mm，P50 8.61 mm，P90 12.01 mm；左 two pads 2.95 N，右 two pads 0.30 N，9.8:1。\n" +
    "几何：膜片平面在 TCP z=−72.29 mm，刀口中心 L_blade=37 mm；绕 TCP 旋转 θ 时，刀口横移 = 37·tanθ，膜片中心横移 = 72.29·tanθ，比值 0.512。\n" +
    "5°：膜片 6.32 mm、刀口 3.24 mm、刀口轴向 0.14 mm。峰值 CoP 8.19 mm 需 6.47°，5° 后残余 1.87 mm；P50 8.61 mm 需 6.79°，5° 后残余 2.29 mm；P90 12.01 mm 需 9.43°，5° 后残余 5.68 mm。\n" +
    "到 2 mm：峰值 4.89°/3.17 mm；P50 5.22°/3.38 mm；P90 7.88°/5.12 mm。\n" +
    "注意：CoP→θ 用一阶杠杆模型（CoP 随压头中心平移），不是接触力学真值；刀口位移是刚体几何精确值。"
  );
}

/* ------------------------------------------------------------------ P4 */
{
  const s = pptx.addSlide();
  s.background = { color: C.ink };
  eyebrow(s, "结论  /  2026-10-23  ·  数据汇报", true);
  text(s, "89.1% 可试运行；CoP 纠偏先限幅，再谈闭环", 0.55, 0.78, 12.2, 0.62, {
    fontSize: 26,
    bold: true,
    color: C.white,
  });

  const cards = [
    ["89.1%", "真机套入成功率", "41 / 46；target_0/1 100%，target_2 80%，target_3 70%", C.teal],
    ["5.24 N", "唯一一次五点同步峰值", "CoP 8.19 mm；左两点 2.95 : 右两点 0.30 N\n172 帧同时 >0.1 N / 4.26 s", C.coral],
    ["3.24 mm", "5° 纠偏的刀口横移", "全回中 P50 6.8° / P90 9.4°；到 2 mm 5.2° / 7.9°\n= 40% 捕获窗口 / 108% 刀面标定", C.peach],
  ];
  cards.forEach((c, i) => {
    const x = 0.55 + i * 4.1;
    card(s, x, 1.62, 3.9, 2.12, C.ink2, true);
    text(s, c[0], x + 0.26, 1.82, 3.35, 0.62, {
      fontSize: 34,
      bold: true,
      color: c[3],
      fontFace: NUM,
    });
    text(s, c[1], x + 0.26, 2.48, 3.35, 0.28, { fontSize: 11, bold: true, color: C.white });
    text(s, c[2], x + 0.26, 2.82, 3.35, 0.72, {
      fontSize: 8.8,
      color: C.dim,
      lineSpacingMultiple: 1.2,
    });
  });

  const acts = [
    ["01", "限幅", "短期 CoP 只记录不闭环；若启用，刀口横移 ≤ 1.5 mm（θ ≤ 2.3°），且径向余量 > 5 mm。"],
    ["02", "门控", "CoP > 8 mm 或 5° 后残余 > 2 mm 不硬转；先回退 / 降速，避免把剪切点推出捕获窗口。"],
    ["03", "补测", "球 / 桃五点同时 + 1 / 2 / 5 mm + 5° 旋转，≥ 10 次；验证符号、收敛与刀口实际位移。"],
    ["04", "采摘", "开 tool 做 level 7；当前采摘确认 0/46。level 6 只是套入，不等于采摘成功；目标 95%。"],
  ];
  acts.forEach((a, i) => {
    const col = i % 2;
    const row = Math.floor(i / 2);
    const x = 0.55 + col * 6.2;
    const y = 4.08 + row * 1.2;
    text(s, a[0], x, y, 0.5, 0.3, { fontSize: 12, bold: true, color: C.peach, fontFace: NUM });
    text(s, a[1], x + 0.48, y - 0.03, 0.8, 0.28, { fontSize: 11.5, bold: true, color: C.white });
    text(s, a[2], x + 1.28, y - 0.03, 4.55, 0.92, {
      fontSize: 9.1,
      color: C.dim,
      lineSpacingMultiple: 1.2,
    });
  });

  text(s, "下一轮验收：套入 ≥ 95% · 五点台架 ≥ 10 次 · 开 tool 做 level 7 切断确认", 0.58, 6.55, 12.1, 0.28, {
    fontSize: 9.4,
    bold: true,
    color: C.coral,
  });

  footer(s, "数据源：session_20261022_163539 / 150712；force_10s、force_20s、force_20s_manual；bite_shear_v1.yaml；force_pad_positions.md。", 4, true);

  s.addNotes(
    "建议分四条：\n" +
    "1) 限幅：现阶段 CoP 只记录，不直接闭环；若启用，刀口横移上限 1.5 mm（θ=atan(1.5/37)=2.3°），并且只有径向余量 >5 mm 才允许。\n" +
    "2) 门控：CoP>8 mm 或 5° 后残余 >2 mm 时不硬转；先回退/降速。P90 CoP 12 mm 需 9.4°，5° 只能压到 5.7 mm，继续转会把刀口推出捕获窗口。\n" +
    "3) 补测：球/桃五点同时 + 1/2/5 mm + 5° 旋转，至少 10 次，验证旋转符号、CoP 收敛、刀口实际位移；当前只有 1 次真机五点记录。\n" +
    "4) 采摘：tool 未开，level 7 = 0；下一轮开 tool 做切断+撤退，目标 95% 套入、可接受的切断确认率。"
  );
}

pptx.writeFile({ fileName: OUT }).then((f) => console.log(f));
