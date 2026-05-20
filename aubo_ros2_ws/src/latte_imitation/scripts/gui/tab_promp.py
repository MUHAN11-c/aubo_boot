"""ProMP推理标签页"""
import os, numpy as np
from scipy.interpolate import interp1d
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from .workers import extract_forming
CARTESIAN_DIR=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),"resource","cartesian","right")
TOP5=[32,25,23,6,8]

class TabPromp(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.gui=parent
        l=QVBoxLayout(self);l.setContentsMargins(0,0,0,0)
        self.fig=Figure(figsize=(16,6),dpi=100);self.canvas=FigureCanvas(self.fig);l.addWidget(self.canvas)
        bar=QHBoxLayout();bar.addWidget(QLabel("示教集:"))
        self.demo_combo=QComboBox();self.demo_combo.addItems(["top5","all"]);bar.addWidget(self.demo_combo)
        self.btn_run=QPushButton("运行推理");bar.addWidget(self.btn_run);bar.addStretch();l.addLayout(bar)
        self.data=None
    def run(self):
        from latte_imitation.promp_learner import ProMP3D
        from latte_imitation.trajectory import CartesianTrajectory
        eps=TOP5 if self.demo_combo.currentText()=="top5" else list(range(40))
        demos=[]
        for ep in eps:
            path=os.path.join(CARTESIAN_DIR,f"episode_{ep:06d}.npz")
            if os.path.exists(path):demos.append(extract_forming(CartesianTrajectory.load(path).positions)[0])
        if not demos:return
        p=ProMP3D(n_basis=20,sigma=0.03);p.learn_multiple(demos);T=p.promp_x._T_learned;cond=p.generate(T=T)
        individuals=[]
        for demo in demos:pi=ProMP3D(n_basis=20,sigma=0.03);pi.learn(demo);individuals.append(pi.generate(T=T))
        stack=np.stack(individuals,axis=0);mean_traj=np.mean(stack,axis=0);std_traj=np.std(stack,axis=0)
        self.data={"demos":demos,"promp":p,"cond":cond,"mean":mean_traj,"std":std_traj,"n":len(demos)}
        return self.data
    def draw(self):
        if self.data is None:return
        d=self.data;self.fig.clear();gs=self.fig.add_gridspec(2,3,hspace=0.4,wspace=0.3);T=len(d["cond"]);idx=np.arange(T)
        for col,(dim,lbl) in enumerate([(0,'X'),(1,'Y'),(2,'Z')]):
            ax=self.fig.add_subplot(gs[0,col])
            ax.fill_between(idx,d["mean"][:,dim]-2*d["std"][:,dim],d["mean"][:,dim]+2*d["std"][:,dim],alpha=0.25,color='green',label='±2σ')
            ax.plot(idx,d["mean"][:,dim],'b-',lw=1.5,label='均值');ax.plot(idx,d["cond"][:,dim],'g-',lw=2.0,label='条件推理')
            for demo in d["demos"]:ax.plot(np.linspace(0,T-1,len(demo)),demo[:,dim],'gray',lw=0.2,alpha=0.4)
            ax.set_title(f'{lbl}维度±2σ');ax.set_xlabel('帧');ax.set_ylabel(f'{lbl}(m)');ax.legend(fontsize=7);ax.grid(alpha=0.3)
        for col,(d1,d2,l1,l2) in enumerate([(0,1,'X','Y'),(0,2,'X','Z')]):
            ax=self.fig.add_subplot(gs[1,col])
            ax.plot(d["cond"][:,d1],d["cond"][:,d2],'g-',lw=2.0,label='条件推理');ax.plot(d["mean"][:,d1],d["mean"][:,d2],'b--',lw=1.5,label='均值')
            for demo in d["demos"]:ax.plot(demo[:,d1],demo[:,d2],'gray',lw=0.3,alpha=0.3)
            ax.set_xlabel(l1);ax.set_ylabel(l2);ax.set_title(f'{l1}{l2}投影');ax.legend(fontsize=7);ax.grid(alpha=0.3);ax.set_aspect('equal')
        ax=self.fig.add_subplot(gs[1,2]);ax.axis('off')
        rows=[['指标','值'],['示教数',str(d["n"])],['σ_X',f'{d["std"][:,0].mean()*1000:.1f}mm'],['σ_Y',f'{d["std"][:,1].mean()*1000:.1f}mm'],['σ_Z',f'{d["std"][:,2].mean()*1000:.1f}mm']]
        tbl=ax.table(cellText=rows,loc='center',cellLoc='center');tbl.auto_set_font_size(False);tbl.set_fontsize(9)
        for i in range(2):tbl[0,i].set_facecolor('#e8e8e8');ax.set_title('推理统计')
        self.canvas.draw_idle()
