#!/usr/bin/env python3
"""5 点力 / CoP 实时网页（默认 8100）。

只订阅 /force/points 与 /force/status，不碰串口；仿真或真机都能看。
页面用 SSE 100 Hz 推送，CoP 按 common.PADS 的 CAD 点位加权。

用法:
  ros2 run serial_force_sim cop_web [--port 8100]
"""
import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt8MultiArray

from serial_force_sim.common import CH, PADS, cop

LOCK = threading.Lock()
STATE = {'f': [0.0] * 5, 'cop': [0.0, 0.0], 'total': 0.0,
         'status': [0] * 5, 'stamp': None}
PADS_JS = json.dumps({str(c): list(PADS[c]) for c in CH})


class Listener(Node):
    def __init__(self):
        super().__init__('cop_web_sim')
        self.create_subscription(
            Float64MultiArray, '/force/points', self.cb, 10)
        self.create_subscription(
            UInt8MultiArray, '/force/status', self.cb_s, 10)

    def cb(self, msg):
        if len(msg.data) != 5:
            return
        f = [float(v) for v in msg.data]
        p, tot = cop(f)
        with LOCK:
            STATE.update(f=f, cop=[float(p[0]), float(p[1])], total=tot,
                         stamp=time.monotonic())

    def cb_s(self, msg):
        if len(msg.data) == 5:
            with LOCK:
                STATE['status'] = [int(v) for v in msg.data]


HTML = r'''<!doctype html><html><head><meta charset="utf-8">
<title>5 点力 · CoP 仿真</title>
<style>
body{margin:0;background:#0d1116;color:#e6edf3;font-family:system-ui,sans-serif}
.wrap{display:flex;gap:24px;padding:24px}
h1{font-size:20px;margin:0 0 8px}
.sub{color:#8b98a3;font-size:12px;margin-bottom:16px}
svg{background:#121820;border:1px solid #24303a;border-radius:12px}
.row{display:flex;gap:12px;flex-wrap:wrap}
.card{background:#121820;border:1px solid #24303a;border-radius:10px;padding:12px 16px;min-width:180px}
.big{font-size:26px;font-variant-numeric:tabular-nums}
.small{color:#8b98a3;font-size:12px}
.bar{height:8px;background:#1e2833;border-radius:4px;margin:4px 0 10px;overflow:hidden}
.fill{height:100%;background:linear-gradient(90deg,#7fd6a0,#ffb648,#ff5a5a);width:0}
</style></head><body><div class="wrap">
<div><h1>5 点力 · CoP 仿真</h1>
<div class="sub">/force/points · 5 点 · 实时 CoP · 模拟数据</div>
<svg id="pad" width="560" height="560" viewBox="0 0 560 560"></svg></div>
<div style="min-width:320px">
<div class="card"><div class="small">总力</div><div class="big" id="total">0.00 N</div>
<div class="small" id="cop">CoP --</div><div class="small" id="stat">等待数据</div></div>
<div id="bars" style="margin-top:16px"></div>
</div></div>
<script>
const PADS=__PADS__;
const CH=[1,2,3,5,7], S=8, C=280;
const svg=document.getElementById('pad');
const NS='http://www.w3.org/2000/svg';
function mk(t,a){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);return e}
// pentagon + points
svg.appendChild(mk('circle',{cx:C,cy:C,r:16.35*S,fill:'none',stroke:'#2b3742','stroke-dasharray':'4 5'}));
svg.appendChild(mk('circle',{cx:C,cy:C,r:3,fill:'#58a6ff'}));
const dots={}, names={};
for(const c of CH){const [x,y]=PADS[c]; const X=C+x*S, Y=C-y*S;
  const g=mk('g',{}); const c0=mk('circle',{cx:X,cy:Y,r:9,fill:'#3a4753',stroke:'#c9d4dd','stroke-width':1});
  const t=mk('text',{x:X,y:Y+4,'text-anchor':'middle','font-size':11,fill:'white','font-weight':'bold'});
  t.textContent=c; g.appendChild(c0); g.appendChild(t); svg.appendChild(g); dots[c]=c0;
}
const copDot=mk('circle',{cx:C,cy:C,r:7,fill:'#ff5a5a',stroke:'white','stroke-width':1.5});
const copLine=mk('line',{x1:C,y1:C,x2:C,y2:C,stroke:'#ff5a5a','stroke-width':1,'stroke-dasharray':'3 3'});
svg.appendChild(copLine); svg.appendChild(copDot);
const lb=mk('text',{x:C+10,y:C-10,fill:'#ff8b8b','font-size':12}); lb.textContent='CoP'; svg.appendChild(lb);
function bar(c){const d=document.createElement('div'); d.innerHTML=`<div class="small">ch${c} <span id="v${c}">0.00 N</span></div><div class="bar"><div class="fill" id="b${c}"></div></div>`; document.getElementById('bars').appendChild(d)}
CH.forEach(bar);
const es=new EventSource('/stream');
es.onmessage=e=>{const d=JSON.parse(e.data);
  document.getElementById('total').textContent=d.total.toFixed(2)+' N';
  document.getElementById('cop').textContent=`CoP (${d.cop[0].toFixed(1)}, ${d.cop[1].toFixed(1)}) mm`;
  document.getElementById('stat').textContent=d.total>0.01?'实时 · 5 点':'空载';
  d.f.forEach((v,i)=>{const c=CH[i]; dots[c].setAttribute('fill', v>0.8?'#ff5a5a':v>0.3?'#ffb648':'#3a4753');
    document.getElementById('v'+c).textContent=v.toFixed(2)+' N';
    document.getElementById('b'+c).style.width=Math.min(100,v/4*100)+'%';});
  const X=C+d.cop[0]*S, Y=C-d.cop[1]*S; copDot.setAttribute('cx',X); copDot.setAttribute('cy',Y);
  copLine.setAttribute('x2',X); copLine.setAttribute('y2',Y);
};
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            b = HTML.replace('__PADS__', PADS_JS).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)
        elif self.path.startswith('/stream'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        d = {'f': STATE['f'], 'cop': STATE['cop'],
                             'total': STATE['total'],
                             'status': STATE['status']}
                    self.wfile.write(
                        f'data: {json.dumps(d)}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8100)
    args = ap.parse_args()
    rclpy.init()
    n = Listener()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(f'CoP web: http://127.0.0.1:{args.port}', flush=True)
    srv.serve_forever()


if __name__ == '__main__':
    main()
