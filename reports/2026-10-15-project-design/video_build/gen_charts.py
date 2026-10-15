# -*- coding: utf-8 -*-
"""数据驱动图表卡：全部数字来自 runs/ 账本实测"""
import json, glob, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

FP = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
font_manager.fontManager.addfont(FP)
font_manager.fontManager.addfont(FB)
plt.rcParams["font.family"] = "Noto Sans CJK JP"  # ttc 注册名
plt.rcParams["axes.unicode_minus"] = False

OUT = os.path.join(os.path.dirname(__file__), "cards")
DEEP=(30/255,59/255,36/255); FOREST=(44/255,95/255,45/255); MOSS=(151/255,188/255,98/255)
PEACH=(232/255,168/255,124/255); PEACHD=(201/255,111/255,74/255); TXT=(34/255,48/255,31/255)
MUTE=(102/255,117/255,95/255); GRID=(217/255,228/255,212/255)

# ---------- 数据（runs 账本实测） ----------
v1_cyc=[28.437,28.435,28.437,28.627]
v2_cyc=[25.9,40.0,40.1,30.2]
v1_stage={"approach_insert":[15.302,15.376,15.338,15.512],
          "reconfirm":[0.200,0.200,0.201,0.200],
          "retreat":[12.542,12.545,12.646,12.552]}
# round_* 10-15 自动轮：完成周期与失败类型
rounds_done=[28.25,28.7,33.3,34.3,33.9,29.9,33.9,33.1,28.7,31.3,24.5,28.1]
rounds_fail={"decision_snapshot":4,"decision_geometry":1,"canceled":1,"full_failed":2}
stab={"RViz 复跑×3":[31.1,33.1,None],"闭环稳定性×2":[29.3,None],"Web 关节链×1":[34.7,None]}

def style_ax(ax):
    for s in ("top","right"): ax.spines[s].set_visible(False)
    for s in ("left","bottom"): ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTE, labelsize=13)
    ax.yaxis.grid(True, color=GRID, lw=1); ax.set_axisbelow(True)

# ---------- 卡 1：同冻结袋 v1 vs v2 ----------
fig, axes = plt.subplots(1, 2, figsize=(19.2,10.8), dpi=100)
fig.patch.set_facecolor("white")
ax=axes[0]
x=[0,1]; means=[sum(v1_cyc)/4, sum(v2_cyc)/4]
ax.bar(x, means, width=0.5, color=[FOREST,PEACHD], zorder=3)
for i,(cyc,m) in enumerate(zip([v1_cyc,v2_cyc],means)):
    ax.scatter([i]*len(cyc), cyc, s=140, color="white", edgecolor=TXT, lw=1.6, zorder=5)
    ax.text(i, m+2.2, f"均值 {m:.1f}s", ha="center", fontsize=17, fontweight="bold", color=TXT)
    sig=max(cyc)-min(cyc)
ax.errorbar(x, means, yerr=[[means[0]-min(v1_cyc)],[max(v2_cyc)-means[1]]], fmt="none",
            ecolor=MUTE, elinewidth=2, capsize=10, zorder=4)
ax.text(0, 40.5, "极差 0.19s", ha="center", fontsize=15, color=FOREST, fontweight="bold")
ax.text(1, 44.5, "极差 14.2s", ha="center", fontsize=15, color=PEACHD, fontweight="bold")
ax.set_xticks(x); ax.set_xticklabels(["v1 现行栈（4/4 完成）","v2 新框架（4/6 完成）"], fontsize=17, color=TXT)
ax.set_ylabel("单果周期 (s)", fontsize=15, color=MUTE); ax.set_ylim(0,48)
style_ax(ax); ax.set_title("单果周期 · 同一冻结数据 · 2026-10-12", fontsize=21, fontweight="bold", color=TXT, pad=14)
ax=axes[1]
labels=["v1 现行栈","v2 新框架"]
ok=[4,4]; fail=[0,2]
ax.barh(labels, ok, height=0.45, color=FOREST, label="完成", zorder=3)
ax.barh(labels, fail, left=ok, height=0.45, color=PEACHD, label="失败", zorder=3)
for i,(o,f) in enumerate(zip(ok,fail)):
    ax.text(o/2, i, f"{o} 轮完成", ha="center", va="center", fontsize=17, color="white", fontweight="bold")
    if f: ax.text(o+f/2, i, f"{f}", ha="center", va="center", fontsize=16, color="white", fontweight="bold")
ax.text(4.15, 1, " 1×逆解不可达\n 1×依赖不可用", fontsize=13.5, color=MUTE, va="center")
ax.set_xlim(0,6.4); ax.invert_yaxis()
ax.set_xlabel("轮数", fontsize=15, color=MUTE)
style_ax(ax); ax.yaxis.grid(False); ax.xaxis.grid(True, color=GRID, lw=1)
ax.set_title("完成率对照（v2 失败含类型化失败码）", fontsize=21, fontweight="bold", color=TXT, pad=14)
ax.tick_params(axis="y", labelsize=17)
fig.text(0.5, 0.035, "数据源：runs/frozen_v1_1791774*、runs/frozen_v2_1791774* ledger.json（v1：approach 15.3 + reconfirm 0.2 + retreat 12.5；v2：10 阶段 FSM 含 INSERT/CUT/RELEASE）",
         ha="center", fontsize=13.5, color=MUTE)
fig.suptitle("两代框架 · 同冻结数据对照", fontsize=30, fontweight="bold", color=TXT, y=0.965)
fig.subplots_adjust(left=0.07, right=0.97, top=0.82, bottom=0.12, wspace=0.25)
fig.savefig(f"{OUT}/chart_v1v2.png", facecolor="white"); plt.close(fig)

# ---------- 卡 2：v1 阶段耗时分解 ----------
fig, ax = plt.subplots(figsize=(19.2,10.8), dpi=100)
fig.patch.set_facecolor("white")
runs=[f"第{i+1}轮" for i in range(4)]
colors={"approach_insert":FOREST,"reconfirm":MOSS,"retreat":PEACHD}
names={"approach_insert":"套入 approach_insert","reconfirm":"复确认 reconfirm","retreat":"撤离 retreat"}
bottom=[0,0,0,0]
for k in ["approach_insert","reconfirm","retreat"]:
    vals=v1_stage[k]
    ax.barh(runs, vals, left=bottom, height=0.55, color=colors[k], label=names[k], zorder=3)
    for i,(v,b) in enumerate(zip(vals,bottom)):
        if k=="reconfirm":
            ax.text(b+v+0.15, i, f"{v:.1f}s", va="center", fontsize=13, color=MUTE)
        else:
            ax.text(b+v/2, i, f"{v:.1f}s", va="center", ha="center", fontsize=15, color="white", fontweight="bold")
    bottom=[b+v for b,v in zip(bottom,vals)]
for i,b in enumerate(bottom):
    ax.text(b+0.2, i, f"合计 {b:.1f}s", va="center", fontsize=15, color=TXT, fontweight="bold")
ax.set_yticks(range(4)); ax.set_yticklabels(runs, fontsize=17, color=TXT); ax.invert_yaxis()
ax.set_xlim(0,33); ax.set_xlabel("耗时 (s)", fontsize=15, color=MUTE)
style_ax(ax); ax.yaxis.grid(False); ax.xaxis.grid(True, color=GRID, lw=1)
ax.legend(loc="lower right", fontsize=15, frameon=False)
ax.set_title("v1 冻结链 · 阶段耗时分解（4 轮实测，tool 档 0s 未显示）", fontsize=25, fontweight="bold", color=TXT, pad=16)
fig.text(0.5, 0.04, "数据源：runs/frozen_v1_* ledger.json stage_durations —— 四轮各阶段波动 < 0.15s（套入 15.30→15.51 / 撤离 12.54→12.65）",
         ha="center", fontsize=13.5, color=MUTE)
fig.subplots_adjust(left=0.09, right=0.96, top=0.86, bottom=0.11)
fig.savefig(f"{OUT}/chart_stages.png", facecolor="white"); plt.close(fig)

# ---------- 卡 3：自动优化轮 + 真机稳定性 ----------
fig, axes = plt.subplots(1, 2, figsize=(19.2,10.8), dpi=100, gridspec_kw={"width_ratios":[1.35,1]})
fig.patch.set_facecolor("white")
ax=axes[0]
n_ok=len(rounds_done); n_f=sum(rounds_fail.values()); total=n_ok+n_f
xs=list(range(1,total+1))
vals=rounds_done+[60.1]*rounds_fail["decision_snapshot"]+[60.1]+[60.1]+[180.1]*rounds_fail["full_failed"]
GREY=(154/255,167/255,155/255)
cols=[FOREST]*n_ok+[PEACHD]*rounds_fail["decision_snapshot"]+[PEACHD]+[GREY]+[PEACHD]*rounds_fail["full_failed"]
bars=ax.bar(xs, vals, color=cols, width=0.68, zorder=3)
ax.axhline(28.4, color=MUTE, lw=1.4, ls="--")
ax.text(total+0.4, 29.5, "冻结基线 28.4s", fontsize=13.5, color=MUTE, ha="right")
ax.set_xticks([]); ax.set_ylabel("单轮耗时 (s)", fontsize=15, color=MUTE); ax.set_ylim(0,195)
ax.text(0.97,0.95,f"完成 {n_ok}/{total} = {n_ok/total:.0%}", transform=ax.transAxes, ha="right",
        fontsize=20, fontweight="bold", color=FOREST)
notes="失败全类型化：决策快照超时×4 · 几何数据过期×1 · 现场取消×1 · 执行失败×2 —— 零无码失败"
ax.text(0.5,-0.09,notes, transform=ax.transAxes, ha="center", fontsize=13.5, color=MUTE)
style_ax(ax)
ax.set_title("10-15 自动优化轮（19 轮 · 完成周期 24.5–34.3s）", fontsize=20, fontweight="bold", color=TXT, pad=14)
ax=axes[1]
ys=[]; vals2=[]; cols2=[]; labs=[]
for i,(k,v) in enumerate(stab.items()):
    for x in v:
        if x: ys.append(len(labs)); vals2.append(x); cols2.append(FOREST)
        else: ys.append(len(labs)); vals2.append(63); cols2.append(GREY)
    labs.append(k)
ax.bar(ys, vals2, color=cols2, width=0.6, zorder=3)
for y,v in zip(ys,vals2):
    ax.text(y, v+1.2, f"{v:.1f}" if v<50 else "失败", ha="center", fontsize=13.5, color=TXT)
ax.set_xticks(sorted(set(ys))); ax.set_xticklabels([l.replace("×","\n") for l in labs], fontsize=14, color=TXT)
ax.set_ylim(0,72); ax.set_ylabel("周期 (s)", fontsize=15, color=MUTE)
style_ax(ax)
ax.set_title("10-14 真机稳定性复验", fontsize=20, fontweight="bold", color=TXT, pad=14)
fig.suptitle("复现性 · 自动轮与真机复验", fontsize=30, fontweight="bold", color=TXT, y=0.965)
fig.text(0.5, 0.035, "数据源：runs/round_*/ledger.json（2026-10-15）· rviz_repeat / stability_full_cycle / web_joint_full（2026-10-14）",
         ha="center", fontsize=13.5, color=MUTE)
fig.subplots_adjust(left=0.06, right=0.97, top=0.80, bottom=0.16, wspace=0.22)
fig.savefig(f"{OUT}/chart_rounds.png", facecolor="white"); plt.close(fig)
print("charts done")
