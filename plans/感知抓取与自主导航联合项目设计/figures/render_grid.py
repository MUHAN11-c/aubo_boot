#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""以像素网格手绘流程图（fig02 批次主流程 / fig04 套入流程）。
样式与 mermaid.json 主题一致：节点 #D6E4F0 / 边 #2F5496 / 标签笺 #FFF2CC。
用法: python3 render_grid.py
"""
from PIL import Image, ImageDraw, ImageFont

TTC = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FILL = "#D6E4F0"
EDGE = "#2F5496"
TEXT = "#1F3864"
CHIP_BG = "#FFF2CC"
SCALE = 2  # 输出 2x，保证印刷清晰


def load_fonts():
    # 在 ttc 中定位简体中文(Style=SC)面的索引
    idx_sc = 0
    for i in range(6):
        try:
            f = ImageFont.truetype(TTC, 26, index=i)
            if "SC" in f.getname()[1]:
                idx_sc = i
                break
        except Exception:
            break
    return {
        "node": ImageFont.truetype(TTC, 26 * SCALE, index=idx_sc),
        "small": ImageFont.truetype(TTC, 20 * SCALE, index=idx_sc),
    }


class Chart:
    def __init__(self, w, h, fonts):
        self.w, self.h = w * SCALE, h * SCALE
        self.im = Image.new("RGB", (self.w, self.h), "white")
        self.d = ImageDraw.Draw(self.im)
        self.fonts = fonts

    def _xy(self, x, y):
        return (x * SCALE, y * SCALE)

    def box(self, x0, y0, x1, y1, text, font="node"):
        d = self.d
        d.rounded_rectangle(
            [self._xy(x0, y0), self._xy(x1, y1)],
            radius=12 * SCALE, fill=FILL, outline=EDGE, width=3 * SCALE,
        )
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        d.text(self._xy(cx, cy), text, font=self.fonts[font], fill=TEXT, anchor="mm")

    def diamond(self, cx, cy, hw, hh, text, font="node"):
        d = self.d
        pts = [(cx, cy - hh), (cx + hw, cy), (cx, cy + hh), (cx - hw, cy)]
        d.polygon([self._xy(px, py) for px, py in pts], fill=FILL, outline=EDGE, width=3 * SCALE)
        d.text(self._xy(cx, cy), text, font=self.fonts[font], fill=TEXT, anchor="mm")

    def chip(self, cx, cy, text, rotated=False):
        """边标签笺(黄色小框)。rotated=True 时为竖排(沿竖直边)。"""
        d = self.d
        font = self.fonts["small"]
        bbox = d.textbbox((0, 0), text, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        pad = 7 * SCALE
        if rotated:
            temp = Image.new("RGBA", (tw + 2 * pad, th + 2 * pad), (0, 0, 0, 0))
            td = ImageDraw.Draw(temp)
            td.rounded_rectangle([0, 0, tw + 2 * pad - 1, th + 2 * pad - 1],
                                 radius=6 * SCALE, fill=CHIP_BG,
                                 outline=EDGE, width=3 * SCALE)
            td.text((pad + tw / 2, pad + th / 2), text, font=font,
                    fill=TEXT, anchor="mm")
            temp = temp.rotate(90, expand=True)
            px, py = self._xy(cx, cy)
            self.im.paste(temp, (int(px - temp.width / 2), int(py - temp.height / 2)), temp)
        else:
            px, py = self._xy(cx, cy)
            x0, y0 = int(px - tw / 2 - pad), int(py - th / 2 - pad)
            d.rounded_rectangle(
                [x0, y0, x0 + tw + 2 * pad, y0 + th + 2 * pad],
                radius=6 * SCALE, fill=CHIP_BG, outline=EDGE, width=3 * SCALE,
            )
            d.text((px, py), text, font=font, fill=TEXT, anchor="mm")

    def arrow(self, pts, chip=None, chip_at=None, rotated_chip=False):
        """pts: [(x,y),...] 折线；末端带箭头。chip: 标签文字, chip_at: (x,y)。"""
        d = self.d
        # 折线
        d.line([self._xy(x, y) for x, y in pts], fill=EDGE, width=3 * SCALE, joint="curve")
        # 箭头
        (x2, y2), (x1, y1) = pts[-1], pts[-2]
        import math
        ang = math.atan2((y2 - y1) * SCALE, (x2 - x1) * SCALE)
        L = 14 * SCALE
        a = 0.46
        p1 = (x2 * SCALE, y2 * SCALE)
        p2 = (x2 * SCALE - L * math.cos(ang - a), y2 * SCALE - L * math.sin(ang - a))
        p3 = (x2 * SCALE - L * math.cos(ang + a), y2 * SCALE - L * math.sin(ang + a))
        d.polygon([p1, p2, p3], fill=EDGE)
        if chip and chip_at:
            self.chip(chip_at[0], chip_at[1], chip, rotated=rotated_chip)

    def save(self, path):
        self.im.save(path)
        print("saved", path)


def fig02(fonts):
    # 图4-1 批次主流程：3列×5行
    c = Chart(1090, 1048, fonts)
    # 行1
    c.box(60, 50, 260, 190, "人工开批")
    c.box(310, 50, 510, 190, "L1 作业位靠近")
    c.box(560, 50, 780, 190, "中止本批")
    # 行2
    c.box(60, 240, 260, 380, "场景观察锁定")
    c.box(310, 240, 510, 380, "选定当前颗")
    c.box(560, 240, 780, 380, "L2 目标靠近")
    # 行3
    c.box(60, 430, 260, 570, "近距重建")
    c.box(310, 430, 510, 570, "套入")
    c.diamond(670, 500, 105, 65, "末端 B？")
    # 行4
    c.box(60, 620, 260, 760, "放弃本颗")
    c.diamond(410, 690, 95, 65, "姿态门")
    c.box(560, 620, 780, 760, "记账")
    # 行5
    c.diamond(160, 880, 100, 70, "下一项")
    c.box(560, 810, 780, 950, "本批结束")

    c.arrow([(260, 120), (310, 120)])
    c.arrow([(510, 120), (560, 120)], chip="失败", chip_at=(505, 95))
    c.arrow([(400, 190), (238, 240)], chip="成功", chip_at=(250, 210))
    c.arrow([(260, 310), (310, 310)])
    c.arrow([(510, 310), (560, 310)])
    c.arrow([(660, 380), (240, 430)], chip="可抓取", chip_at=(560, 402))
    c.arrow([(260, 500), (310, 500)], chip="有许可", chip_at=(270, 470))
    c.arrow([(160, 570), (160, 620)], chip="无许可", chip_at=(95, 594))
    c.arrow([(310, 560), (235, 620)], chip="失败", chip_at=(290, 592))
    c.arrow([(510, 500), (565, 500)])
    c.arrow([(670, 565), (670, 620)], chip="否 方案A", chip_at=(703, 592))
    c.arrow([(608, 545), (468, 643)], chip="是", chip_at=(545, 590))
    c.arrow([(505, 690), (560, 690)], chip="通过", chip_at=(512, 662))
    c.arrow([(315, 690), (260, 690)], chip="不通过", chip_at=(310, 664))
    c.arrow([(160, 760), (160, 810)])
    c.arrow([(660, 760), (213, 844)])
    c.arrow([(260, 880), (560, 880)], chip="结束", chip_at=(410, 852))
    # 回边：下一颗 → 选定当前颗（左侧上绕，再经行1/行2 间隙）
    c.arrow([(60, 880), (42, 880), (42, 210), (285, 210), (285, 310), (310, 310)],
            chip="下一颗", chip_at=(40, 545), rotated_chip=True)
    # 回边：再观察 → 场景观察锁定（最左，底部绕过）
    c.arrow([(160, 950), (160, 1000), (20, 1000), (20, 420), (160, 420), (160, 380)],
            chip="再观察", chip_at=(20, 700), rotated_chip=True)
    c.save("/home/mu/Desktop/aubo_e5_jazzy_ws/plans/感知抓取与自主导航联合项目设计/figures/fig02_批次主流程.png")


def fig04(fonts):
    # 图4-3 套入工序：许可(左) → 套入(中/右)
    c = Chart(1046, 776, fonts)
    # r1
    c.box(60, 40, 260, 180, "车辆可抓取")
    c.box(310, 40, 510, 180, "近距观察建模")
    c.diamond(660, 110, 100, 65, "抓取许可")
    # r2
    c.box(60, 230, 260, 370, "不接触")
    c.box(310, 230, 510, 370, "沿许可轴接近")
    c.diamond(660, 300, 100, 65, "末端")
    # r3
    c.box(60, 420, 260, 560, "采集圆柱零位")
    c.box(310, 420, 510, 560, "推进并读姿态")
    c.diamond(660, 490, 100, 65, "轴偏差")
    c.box(806, 420, 1026, 560, "按行程套入或撤退")
    # r4
    c.box(60, 610, 260, 750, "同轴撤退放弃本颗")
    c.diamond(410, 680, 95, 65, "行程到达")

    c.arrow([(260, 110), (310, 110)])
    c.arrow([(510, 110), (560, 110)])
    c.arrow([(610, 155), (255, 232)], chip="不成立", chip_at=(445, 185))
    c.arrow([(660, 172), (415, 232)], chip="成立", chip_at=(545, 205))
    c.arrow([(510, 300), (560, 300)])
    # 末端A：右侧短绕行 → 按行程套入或撤退
    c.arrow([(760, 300), (792, 300), (792, 490), (806, 490)], chip="A", chip_at=(752, 345))
    # 末端B：斜下 → 采集圆柱零位
    c.arrow([(660, 362), (252, 422)], chip="B", chip_at=(430, 395))
    c.arrow([(260, 490), (310, 490)])
    c.arrow([(510, 490), (560, 490)])
    # 轴偏差 → 可接受 → 行程到达（沿下缘斜线）；→ 超门或突变 → 同轴撤退
    c.arrow([(660, 555), (472, 665)], chip="可接受", chip_at=(555, 608))
    c.arrow([(620, 542), (230, 618)], chip="超门或突变", chip_at=(330, 596))
    # 行程到达 → 否 → 推进并读姿态（循环）；→ 是 → 按行程套入或撤退（底部绕行）
    c.arrow([(410, 615), (410, 560)], chip="否", chip_at=(432, 590))
    c.arrow([(505, 680), (535, 680), (535, 745), (910, 745), (910, 560)],
            chip="是", chip_at=(585, 718))
    c.save("/home/mu/Desktop/aubo_e5_jazzy_ws/plans/感知抓取与自主导航联合项目设计/figures/fig04_套入流程.png")


if __name__ == "__main__":
    fonts = load_fonts()
    fig02(fonts)
    fig04(fonts)
