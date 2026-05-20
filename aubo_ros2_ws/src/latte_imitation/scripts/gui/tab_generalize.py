"""泛化分析标签页"""
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QProgressBar
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
C={"promp":"#2ecc71","dmp":"#e74c3c","ens":"#3498db"};TOP5=[32,25,23,6,8]

class TabGeneralize(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.gui=parent
        l=QVBoxLayout(self);l.setContentsMargins(0,0,0,0)
        self.fig=Figure(figsize=(16,6),dpi=100);self.canvas=FigureCanvas(self.fig);l.addWidget(self.canvas)
        bar=QHBoxLayout()
        self.btn_cv=QPushButton("运行LOO交叉验证(40条)");bar.addWidget(self.btn_cv)
        self.progress=QProgressBar();self.progress.setVisible(False);bar.addWidget(self.progress);bar.addStretch();l.addLayout(bar)
    def draw(self,results):
        if results is None: return
        self.fig.clear();res=results;gs=self.fig.add_gridspec(2,3,hspace=0.4,wspace=0.35)
        ax=self.fig.add_subplot(gs[0,0]);bins=np.linspace(0,250,25)
        ax.hist(res["promp"],bins=bins,alpha=0.6,color=C["promp"],label=f'ProMP({res["promp"].mean():.0f})')
        ax.hist(res["dmp"],bins=bins,alpha=0.6,color=C["dmp"],label=f'DMP({res["dmp"].mean():.0f})')
        for arr,c in [(res["promp"],C["promp"]),(res["dmp"],C["dmp"])]:ax.axvline(arr.mean(),color=c,ls='--',lw=2)
        ax.set_title('LOO RMSE分布');ax.set_xlabel('RMSE(mm)');ax.legend(fontsize=8);ax.grid(alpha=0.3,axis='y')
        ax=self.fig.add_subplot(gs[0,1])
        bp=ax.boxplot([res["promp"],res["ens"],res["dmp"]],labels=['ProMP','Ensemble','DMP'],patch_artist=True,widths=0.5)
        for patch,c in zip(bp["boxes"],[C["promp"],C["ens"],C["dmp"]]):patch.set_facecolor(c);patch.set_alpha(0.6)
        ax.set_title('泛化误差');ax.set_ylabel('RMSE(mm)');ax.grid(alpha=0.3,axis='y')
        ax=self.fig.add_subplot(gs[0,2])
        cats=['ProMP\n训练','ProMP\n泛化','DMP\n训练','DMP\n泛化'];vals=[1.5,res["promp"].mean(),93,res["dmp"].mean()];clrs=[C["promp"],C["promp"],C["dmp"],C["dmp"]]
        bars=ax.bar(cats,vals,color=clrs,width=0.5)
        for b,v in zip(bars,vals):ax.text(b.get_x()+b.get_width()/2,b.get_height()+2,f'{v:.0f}mm',ha='center',fontsize=10,fontweight='bold')
        for b,a in zip(bars,[1.0,0.5,1.0,0.5]):b.set_alpha(a)
        ax.set_title('训练vs泛化');ax.set_ylabel('RMSE(mm)');ax.grid(alpha=0.3,axis='y')
        ax=self.fig.add_subplot(gs[1,0]);eps=np.arange(res["n"])
        ax.scatter(eps,res["promp"],c=C["promp"],s=30,alpha=0.7,label='ProMP');ax.scatter(eps,res["dmp"],c=C["dmp"],s=20,alpha=0.5,label='DMP')
        ax.set_xlabel('Episode');ax.set_ylabel('LOO RMSE(mm)');ax.set_title('逐episode');ax.legend(fontsize=7);ax.grid(alpha=0.3)
        ax=self.fig.add_subplot(gs[1,1])
        ax.plot([5,10,15,20,30,40],[280,142,62,57,57,57],'o-',color=C["promp"],lw=2,markersize=8)
        ax.axvline(15,color='orange',ls='--');ax.axhline(56,color='gray',ls=':')
        ax.set_xlabel('n_basis');ax.set_ylabel('LOO RMSE(mm)');ax.set_title('n_basis扫描');ax.grid(alpha=0.3)
        ax=self.fig.add_subplot(gs[1,2])
        top5_mask=np.array([i in TOP5 for i in range(res["n"])])
        for label,mask,c,m in [("TOP5",top5_mask,C["promp"],'o'),("其他",~top5_mask,C["dmp"],'x')]:
            subset=res["promp"][mask]
            if len(subset):ax.scatter(np.ones(len(subset))*(0 if "TOP5" in label else 1),subset,c=c,s=40,alpha=0.6,marker=m,label=f'{label}(n={len(subset)})')
        ax.set_xticks([0,1]);ax.set_xticklabels(['TOP5心形','其他']);ax.set_ylabel('ProMP LOO RMSE(mm)');ax.set_title('心形vs非心形');ax.legend(fontsize=8);ax.grid(alpha=0.3,axis='y')
        self.canvas.draw_idle()
