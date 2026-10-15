# -*- coding: utf-8 -*-
"""生成对比视频用静态卡与角标（1920x1080，Noto Sans CJK）"""
from PIL import Image, ImageDraw, ImageFont
import os

OUT = os.path.join(os.path.dirname(__file__), "cards")
os.makedirs(OUT, exist_ok=True)
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
def font(path, size, idx=2):
    return ImageFont.truetype(path, size, index=idx)

DEEP=(30,59,36); FOREST=(44,95,45); MOSS=(151,188,98); PEACH=(232,168,124)
PEACHD=(201,111,74); TXT=(34,48,31); MUTE=(102,117,95); TINT=(243,247,241); LINE=(217,228,212)
W,H=1920,1080

def rr(d, box, r, fill):
    d.rounded_rectangle(box, radius=r, fill=fill)

# ---------- 1. 片头卡 ----------
im = Image.new("RGB",(W,H),DEEP); d=ImageDraw.Draw(im)
d.text((W//2,300),"PEACH · 2026-08 → 2026-10",font=font(FR,40),fill=MOSS,anchor="mm")
d.text((W//2,420),"桃采摘机器人 · 优化前后对比",font=font(FB,108),fill=(255,255,255),anchor="mm")
d.text((W//2,540),"感知 · 轨迹 · 测试体系 —— 205 笔提交的工程演进",font=font(FR,46),fill=PEACH,anchor="mm")
stats=[("205","笔提交"),("71.3%","轨迹通过率"),("28.4s","单果周期"),("517","测试全绿")]
for i,(n,l) in enumerate(stats):
    x=W//2-720+i*480
    d.text((x,700),n,font=font(FB,72),fill=MOSS,anchor="mm")
    d.text((x,775),l,font=font(FR,32),fill=(191,211,184),anchor="mm")
im.save(f"{OUT}/card_title.png")

# ---------- 2. 数据对比卡 ----------
im = Image.new("RGB",(W,H),(255,255,255)); d=ImageDraw.Draw(im)
d.text((140,80),"两个月 · 关键指标对比",font=font(FB,64),fill=TXT,anchor="lm")
d.text((140,160),"数据源：git 提交记录 + runs/ 测试账本（ledger / 矩阵基线 / 回放塔）",font=font(FR,30),fill=MUTE,anchor="lm")
cols=[(140,560,"维度"),(720,560,"优化前"),(1330,600,"优化后")]
rows=[
 ("接近轨迹通过率","v2 斜直线证伪：1/30","v4 世界垂直入冠：71.3%（网格 18/20）"),
 ("轨迹连续性","PTP 分段停点","Pilz sequence blend 冠内速度连续"),
 ("袋长估计","系统性偏短 −29%","2D+3D 融合 + 检测框硬限幅修正"),
 ("几何核耗时","全量 RANSAC 打分","两段式 preemptive：512 粗排 / 16 决赛"),
 ("测试规模","少量纯核 pytest","517 测 0 败 + E2E 64/64 + launch_testing"),
 ("感知基线","无冻结基线","852 帧矩阵首轮 GATE PASS 冻结"),
]
y0=230; rh=118
# header
rr(d,(110,y0-8,1810,y0+72),14,DEEP)
d.text((140,y0+32),"维度",font=font(FB,40),fill=(255,255,255),anchor="lm")
d.text((720,y0+32),"优化前",font=font(FB,40),fill=(255,255,255),anchor="lm")
d.text((1330,y0+32),"优化后",font=font(FB,40),fill=(255,255,255),anchor="lm")
for ri,(a,b,c) in enumerate(rows):
    y=y0+72+ri*rh+8
    if ri%2==1:
        rr(d,(110,y,1810,y+rh-8),10,TINT)
    d.text((140,y+rh//2-6),a,font=font(FB,36),fill=TXT,anchor="lm")
    d.text((720,y+rh//2-6),b,font=font(FR,34),fill=MUTE,anchor="lm")
    d.text((1330,y+rh//2-6),c,font=font(FR,34),fill=FOREST,anchor="lm")
d.text((960,1010),"优化后数据：提交 4cc2e5e/781971b（轨迹）· d6051d4（袋长）· bebfdb4（几何核）· 4ba0631（852 帧基线）",
       font=font(FR,26),fill=MUTE,anchor="mm")
im.save(f"{OUT}/card_data.png")

# ---------- 3. 片尾卡 ----------
im = Image.new("RGB",(W,H),DEEP); d=ImageDraw.Draw(im)
d.text((W//2,400),"设计定型 · 工程闭环 · 田间待验",font=font(FB,96),fill=(255,255,255),anchor="mm")
d.text((W//2,530),"PEACH 自主采摘机器人 · 2026-10",font=font(FR,44),fill=MOSS,anchor="mm")
d.text((W//2,640),"素材：runs/compare_rviz_20261012 · peach_stereo 测试报告 · harvest 会话掩膜序列",
       font=font(FR,30),fill=(143,169,138),anchor="mm")
im.save(f"{OUT}/card_end.png")

# ---------- 4. 角标（透明底） ----------
def label(name, text):
    im = Image.new("RGBA",(W,300),(0,0,0,0)); d=ImageDraw.Draw(im)
    d.rounded_rectangle((80,150,1600,270),radius=22,fill=(18,32,20,205))
    d.ellipse((120,178,172,230),fill=PEACHD)
    d.text((200,210),text,font=font(FB,52),fill=(255,255,255),anchor="lm")
    im.save(f"{OUT}/{name}.png")

label("label_s2","① 感知优化 — 立体深度前后对比")
label("label_s3","② 运动执行 — 冻结链完整周期 28.4s")
label("label_s4","③ 相机流点云（左）· SAM 掩膜 712 帧（右）")
print("cards done:", os.listdir(OUT))
