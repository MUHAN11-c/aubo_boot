#!/usr/bin/env python3
"""Render source-reviewed architectural views (not exhaustive ROS connection graphs).

References: Autoware architecture overview, MoveIt Concepts, ros2_control Jazzy
Getting Started. Graphviz neato with fixed layout keeps v1/v2 directly comparable.
No ROS processes or hardware are started. Requires Graphviz and Noto Sans CJK SC.
"""
from pathlib import Path
from html import escape
import xml.etree.ElementTree as ET
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1] / 'docs' / 'images'
COLORS = {'task': '#E9EDFC', 'facts': '#DDF3F1', 'motion': '#FFF0DD',
          'infra': '#EAF0F6', 'hw': '#E5F1E9', 'safety': '#FBE5E8'}
ACCENTS = {'task': '#485BA9', 'facts': '#197C79', 'motion': '#AE7029',
           'infra': '#536A82', 'hw': '#437D58', 'safety': '#B54A5B'}


class Architecture:
    def __init__(self, version, kind):
        self.version, self.kind = version, kind
        title = f'PEACH v{version}  |  {"软件" if kind == "software" else "硬件"}架构'
        self.lines = [
            'digraph G {',
            'graph [layout=neato, overlap=true, splines=polyline, bgcolor="#F7F9FC", '
            'pad=0.35, fontname="Noto Sans CJK SC", labelloc=t, fontsize=28, '
            f'label={json.dumps(title, ensure_ascii=False)}];',
            'node [shape=plain, fontname="Noto Sans CJK SC", fontsize=15, fontcolor="#182B43"];',
            'edge [fontname="Noto Sans CJK SC", fontsize=11, color="#7D8FA5", '
            'fontcolor="#556779", penwidth=1.6, arrowsize=0.65];',
        ]
        self.positions = {}

    def layer(self, ident, y, label, modules, color, x=620, width=840):
        """A layer owns a set of equal-granularity architectural components."""
        cell_width = int(width / len(modules))
        head = (f'<TR><TD COLSPAN="{len(modules)}" BGCOLOR="{COLORS[color]}" '
                f'HEIGHT="38" ALIGN="LEFT"><FONT COLOR="{ACCENTS[color]}" POINT-SIZE="15">'
                f'<B>  {escape(label)}</B></FONT></TD></TR>')
        cells = []
        for title, responsibility, source in modules:
            card = ('<TABLE BORDER="1" COLOR="#DCE4ED" STYLE="ROUNDED" '
                    'CELLBORDER="0" CELLSPACING="0" CELLPADDING="7" BGCOLOR="white">'
                    f'<TR><TD WIDTH="{cell_width-20}" HEIGHT="27"><FONT POINT-SIZE="15" COLOR="#20344D">'
                    f'<B>{escape(title or " ")}</B></FONT></TD></TR>'
                    f'<TR><TD><FONT POINT-SIZE="12" COLOR="#465B73">'
                    f'{escape(responsibility).replace(chr(10), "<BR/>")}</FONT></TD></TR>'
                    f'<TR><TD HEIGHT="23"><FONT POINT-SIZE="10" COLOR="#778AA0">'
                    f'{escape(source or " ")}</FONT></TD></TR></TABLE>')
            cells.append(f'<TD WIDTH="{cell_width}" BGCOLOR="#F2F5F9">{card}</TD>')
        table = ('<TABLE BORDER="1" COLOR="#D5DFE9" STYLE="ROUNDED" CELLBORDER="0" '
                 'CELLSPACING="0" CELLPADDING="9">' + head + '<TR>' + ''.join(cells) + '</TR></TABLE>')
        self.lines.append(f'{ident} [pos="{x},{y}!", label=<{table}>];')
        self.positions[ident] = (x, y)

    def note(self, ident, x, y, title, body, color='infra', width=180):
        table = (f'<TABLE BORDER="1" COLOR="#D5DFE9" STYLE="ROUNDED" CELLBORDER="0" '
                 f'CELLSPACING="0" CELLPADDING="11" BGCOLOR="white">'
                 f'<TR><TD WIDTH="{width}" HEIGHT="33" BGCOLOR="{COLORS[color]}">'
                 f'<FONT COLOR="{ACCENTS[color]}" POINT-SIZE="13"><B>{escape(title)}</B></FONT></TD></TR>'
                 f'<TR><TD HEIGHT="66"><FONT POINT-SIZE="12" COLOR="#53677E">'
                 f'{escape(body).replace(chr(10), "<BR/>")}</FONT></TD></TR></TABLE>')
        self.lines.append(f'{ident} [pos="{x},{y}!", label=<{table}>];')
        self.positions[ident] = (x, y)

    def edge(self, a, b, label='', dashed=False, red=False, port='', route=None, label_at=None, color=None, bidirectional=False):
        attrs = [f'label={json.dumps(label, ensure_ascii=False)}']
        if bidirectional: attrs.append('dir=both')
        if color: attrs += [f'color="{color}"', f'fontcolor="{color}"']
        if route:
            points = [route[0]]
            for p0, p1 in zip(route, route[1:]):
                points += [(p0[0]+(p1[0]-p0[0])/3, p0[1]+(p1[1]-p0[1])/3),
                           (p0[0]+2*(p1[0]-p0[0])/3, p0[1]+2*(p1[1]-p0[1])/3), p1]
            coords = ' '.join(f'{x},{y}' for x, y in points)
            attrs += [f'pos="e,{route[-1][0]},{route[-1][1]} {coords}"', 'headclip=false', 'tailclip=false']
            if label_at: attrs.append(f'lp="{label_at[0]},{label_at[1]}"')
        if dashed: attrs.append('style=dashed')
        if red: attrs += ['color="#B34450"', 'fontcolor="#B34450"']
        if port: attrs += ['tailport=s', 'headport=n']
        self.lines.append(f'{a} -> {b} [{", ".join(attrs)}];')

    def save(self):
        base = ROOT / f'peach_v{self.version}_{self.kind}'
        self.lines.append('}')
        base.with_suffix('.dot').write_text('\n'.join(self.lines), encoding='utf-8')
        layout = json.loads(subprocess.check_output(['neato', '-n2', '-Tjson', str(base.with_suffix('.dot'))]))
        bounds = [float(v) for v in layout['bb'].split(',')]
        mx = ET.Element('mxfile', host='app.diagrams.net')
        page = ET.SubElement(mx, 'diagram', name=f'Peach v{self.version} {self.kind}')
        model = ET.SubElement(page, 'mxGraphModel', page='1', pageWidth=str(int(bounds[2]+100)), pageHeight=str(int(bounds[3]+100)))
        root = ET.SubElement(model, 'root')
        ET.SubElement(root, 'mxCell', id='0')
        ET.SubElement(root, 'mxCell', id='1', parent='0')
        title = ET.SubElement(root, 'mxCell', id='title', parent='1', vertex='1',
            value=f'PEACH v{self.version} | {"软件" if self.kind == "software" else "硬件"}架构',
            style='text;html=1;fontFamily=Noto Sans CJK SC;fontSize=28;fontColor=#182B43;align=center;')
        ET.SubElement(title, 'mxGeometry', x='30', y='0', width=str(bounds[2]), height='40', **{'as':'geometry'})
        for obj in layout['objects']:
            x, y = map(float, obj['pos'].split(','))
            w, h = float(obj['width'])*72, float(obj['height'])*72
            cell = ET.SubElement(root, 'mxCell', id=str(obj['_gvid']+2), value=obj['label'],
                style='shape=rectangle;html=1;whiteSpace=wrap;fillColor=none;strokeColor=none;', vertex='1', parent='1')
            ET.SubElement(cell, 'mxGeometry', x=str(x-w/2+30), y=str(bounds[3]-y-h/2+50), width=str(w), height=str(h), **{'as':'geometry'})
        for i, obj in enumerate(layout.get('edges', [])):
            style='edgeStyle=none;html=1;endArrow=block;strokeColor='+obj.get('color','#556779')+';'
            if obj.get('dir') == 'both': style += 'startArrow=block;'
            if obj.get('style') == 'dashed': style += 'dashed=1;'
            cell=ET.SubElement(root,'mxCell',id='edge'+str(i),value=obj.get('label',''),style=style,edge='1',parent='1',source=str(obj['tail']+2),target=str(obj['head']+2))
            geo = ET.SubElement(cell,'mxGeometry',relative='1',**{'as':'geometry'})
            coords = [pair for pair in obj.get('pos', '').split() if not pair.startswith(('e,','s,'))]
            if len(coords) > 4:
                points = ET.SubElement(geo, 'Array', **{'as':'points'})
                for pair in coords[3:-1:3]:
                    px, py = map(float, pair.split(','))
                    ET.SubElement(points,'mxPoint',x=str(px+30),y=str(bounds[3]-py+50))
        ET.ElementTree(mx).write(base.with_suffix('.drawio'), encoding='utf-8', xml_declaration=True)
        for fmt in ('svg', 'png', 'pdf'):
            subprocess.run(['neato', '-n2', f'-T{fmt}', '-Gdpi=180',
                            str(base.with_suffix('.dot')), '-o', str(base.with_suffix('.'+fmt))], check=True)



def software(v):
    g = Architecture(v, 'software')
    g.layer('operator',900,'任务入口', [('操作员','作业意图 / 使能 / 恢复确认','RunHarvest' if v == 1 else 'RunBatch')], 'infra', x=1370,width=320)
    g.layer('sensor',660,'输入', [('视觉与机器人状态','RGB-D / 内参 / 关节状态','精确图像时刻 TF')], 'infra', x=200,width=290)
    g.layer('per',660,'感知', [('目标观测','YOLO / MobileSAM / 几何与跟踪','peach_scene_perception_node' if v == 1 else 'peach2_perception')], 'facts', x=585,width=310)
    g.layer('model',660,'目标模型与许可', [('几何与作业约束','ICP / TSDF / refit' if v == 1 else '多视点 Huber 融合 / 不确定度', 'target_reconstruction' if v == 1 else 'peach2_target_model + core')], 'facts', x=985,width=330)
    g.layer('task',660,'任务决策', [('批次编排','选果 / 重试 / 账本 / 恢复','peach_supervisor / FSM' if v == 1 else 'peach2_task / BT.CPP')], 'task', x=1370,width=320)
    g.layer('scene',390,'环境模型', [('障碍快照','深度 / 碰撞对象 / 目标滤除','peach_scene_obstacles' if v == 1 else 'peach2_scene')], 'facts',x=985,width=330)
    g.layer('exec',390,'作业执行', [('运动技能与命令门','预抓取 / 套入 / 剪切 / 撤出','peach_arm / ExecutionAuthority' if v == 1 else 'peach2_manipulation / CommandGate'),('末端控制','开合状态 / IO / 剪切确认','arm 内部逻辑' if v == 1 else 'peach2_end_effector 插件')], 'motion',x=1370,width=340)
    g.layer('platform',85,'运动基础设施', [('MoveIt 2','运动学 / 规划 / 碰撞检查','OMPL / Pilz' + (' / MTC' if v == 1 else ''))], 'infra',x=1370,width=320)
    g.layer('driver',85,'硬件抽象', [('控制器与 AUBO 驱动','轨迹执行 / IO / 状态','ros2_control · 驱动只读')], 'infra',x=985,width=330)
    g.layer('physical',-160,'执行设备', [('机械臂与末端','AUBO E5 / 三种剪切工具','real / mock 替代')], 'hw',x=985,width=330)
    g.note('support',585,390,'公共支撑','ROS 2 / IDL / tf2 / URDF\n生命周期 / 参数 / 诊断\n会话记录与标定',width=310)
    g.note('version',585,85,'版本部署边界','brain 三节点同进程\nIMU / Servo 条件支路\n8090 随整栈启动' if v == 1 else '五个能力节点托管\n末端插件在执行进程内\n8091 独立启动',width=310)
    g.note('safety',1370,-160,'独立功能安全','柜体 / 示教器 / 硬件急停\n不经 ROS，不依赖上位机','safety',width=320)
    g.edge('operator','task','作业意图',color='#AA702F')
    g.edge('sensor','per','图像 / 状态',color='#267F89')
    g.edge('per','model','目标观测',color='#267F89')
    g.edge('model','task','模型 / 许可',color='#267F89')
    g.edge('task','exec','作业命令',color='#AA702F')
    g.edge('scene','platform','碰撞场景',color='#267F89')
    g.edge('exec','platform','规划 / 运动请求',color='#AA702F')
    g.edge('platform','driver','轨迹',color='#AA702F')
    g.edge('driver','physical','轨迹 / IO',color='#AA702F')
    g.edge('exec','driver','工具 IO',color='#AA702F',route=[(1550,340),(1720,340),(1720,-40),(985,-40),(985,5)],label_at=(1660,10))
    g.edge('safety','physical','安全回路',red=True)
    g.edge('model','scene','目标滤除',color='#267F89')
    g.edge('support','scene','基础服务',dashed=True)
    g.edge('per','task','观测 / 候选',color='#267F89',route=[(585,740),(585,785),(1250,785),(1250,740)],label_at=(930,799))
    g.edge('sensor','scene','深度',color='#267F89',route=[(200,580),(200,485),(985,485),(985,470)],label_at=(560,498))
    if v == 2:
        g.edge('exec','model','许可查询',dashed=True,route=[(1370,505),(1370,550),(1160,550),(1090,580)],label_at=(1270,563))
    g.edge('exec','task','结果 / 恢复状态',color='#267F89',route=[(1540,390),(1620,390),(1620,660),(1530,660)],label_at=(1650,530))
    g.edge('driver','sensor','状态反馈',color='#267F89',route=[(825,85),(800,85),(800,-30),(30,-30),(30,660),(50,660)],label_at=(430,-17))
    g.note('caption',870,-340,'图例与阅读边界','蓝线：观测、模型、状态 · 橙线：任务与执行命令 · 虚线：支撑或查询 · 红线：硬件安全\n功能架构依据源码快照 2026-10-09；图中框表示能力，不等同于独立进程。',width=1350)
    g.save()


def hardware(v):
    g = Architecture(v, 'hardware')
    g.layer('nx',570,'现场主控 · 默认部署目标',[('Jetson Orin NX 16GB','感知 / 建模 / 调度 / MoveIt / 命令门','整机栈承载；目标机部署验收待完成')], 'task',x=760,width=420)
    g.layer('camera',570,'视觉设备',[('腕载 RGB-D 相机','Percipio PS800-E1 / 双目前端','默认网络直连 NX')], 'facts',x=230,width=310)
    g.layer('rk',810,'可选视觉分流 · 未实装',[('RK3588','采集 / 同步 / 深度与 ROI 预处理','只提供视觉数据，不控制臂或 IO')], 'infra',x=230,width=310)
    g.layer('operator',810,'操作入口',[('操作员终端','作业指令 / 监视 / 授权','应用层人机接口')], 'infra',x=760,width=420)
    g.layer('cabinet',570,'机器人控制',[('AUBO 控制柜','运动控制 / 关节状态 / 工具 IO','网络 · SDK 通信')], 'motion',x=1280,width=310)
    g.layer('arm',320,'机械臂',[('AUBO E5','固定座 / 六关节 / 腕部安装','关节驱动与编码器反馈')], 'hw',x=1280,width=310)
    g.layer('tool',70,'末端执行器',[('可换套袋剪切工具','shear / bite_shear / adaptive_shear','柜体或工具 IO 控制')], 'hw',x=1280,width=310)
    g.layer('imu',320,'末端姿态观测',[('IMU','serial_imu 条件启动' if v == 1 else 'v2 整栈未接入 IMU / Servo','串口 / USB UART')], 'facts',x=230,width=310)
    g.note('safety',1280,810,'独立功能安全','示教器 / 急停 / 安全回路\n人工上电与复位\n不依赖 NX、RK3588 或 ROS','safety',width=310)
    g.note('dev',760,70,'开发与验证环境','Ubuntu / RTX 3090 · 开发 / 仿真 / HIL\n测试时承担主控角色，非现场计算板\nNX 部署与 RK3588 分流均须独立验证',width=420)
    g.note('assembly',760,320,'机械安装边界','固定座 → AUBO E5 → 腕部相机与末端\n三种剪切手是替换关系，不同时安装\n独立刀位反馈 / 剪断第二判据接线待核实',width=420)
    g.edge('camera','nx','默认：GbE / RGB-D',color='#267F89')
    g.edge('camera','rk','可选分流',dashed=True,color='#267F89')
    g.edge('rk','nx','GbE / 预处理数据',dashed=True,color='#267F89',route=[(390,810),(490,810),(490,645),(550,645)],label_at=(495,738))
    g.edge('operator','nx','任务与使能',color='#AA702F')
    g.edge('nx','cabinet','网络：轨迹 / IO / 状态',color='#AA702F',bidirectional=True)
    g.edge('cabinet','arm','关节控制 / 反馈',color='#AA702F',bidirectional=True)
    g.edge('safety','cabinet','硬件安全回路',red=True)
    g.edge('arm','tool','腕部机械连接',color='#65778B')
    g.edge('imu','nx','姿态输入',dashed=True,color='#267F89')
    g.edge('cabinet','tool','DO / DI',color='#AA702F',bidirectional=True,route=[(1440,570),(1510,570),(1510,70),(1440,70)],label_at=(1540,320))
    g.note('caption',800,-150,'图例与部署边界','蓝线：观测数据 · 橙线：设备命令与反馈 · 灰线：机械连接 · 虚线：预留或条件接入 · 红线：硬件安全\nNX / RK3588 角色依据方案 §3.4；设备接口与安装依据源码。非电气原理图，未推定供电或引脚。',width=1400)
    g.save()


if __name__ == '__main__':
    for version in (1, 2):
        hardware(version)
        software(version)
