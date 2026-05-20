"""单条训练标签页"""
import numpy as np
from scipy.signal import savgol_filter, spectrogram
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
DT=0.05;C={"orig":"#1a1a1a","promp":"#2ecc71","dmp":"#e74c3c","ens":"#3498db","form":"#f39c12"}

class TabTrain(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent);self.gui=parent
        l=QVBoxLayout(self);l.setContentsMargins(0,0,0,0)
        self.fig=Figure(figsize=(16,7),dpi=100);self.canvas=FigureCanvas(self.fig);l.addWidget(self.canvas)

    def draw(self,data,show_orig=True,show_promp=True,show_ens=True,ensemble=None):
        if data is None or data.get("recon_p") is None: return
        self.fig.clear();d=data;gs=self.fig.add_gridspec(2,3,hspace=0.4,wspace=0.35,height_ratios=[1,1.1])
        fm,rp=d["form_raw"],d["recon_p"]
        ax=self.fig.add_subplot(gs[0,0]);t=np.arange(d["Tf"])*DT;zs=savgol_filter(d["pos_full"][:,2],21,3)
        ax.plot(t,zs,'gray',lw=1.0,alpha=0.5,label='Z');ax.plot(t,d["pos_full"][:,0],'blue',lw=0.8,alpha=0.3,label='X')
        ax.axvspan(d["fs"]*DT,d["fe"]*DT,alpha=0.12,color=C["form"],label='成形段')
        ax.set_title(f'ep{d["ep"]}');ax.set_xlabel('秒');ax.legend(fontsize=7);ax.grid(alpha=0.3)
        ax3=self.fig.add_subplot(gs[0,1],projection='3d')
        rx=(fm[:,0].min()-0.005,fm[:,0].max()+0.005);ry=(fm[:,1].min()-0.005,fm[:,1].max()+0.005);rz=(fm[:,2].min()-0.005,fm[:,2].max()+0.005)
        if show_orig:ax3.plot(fm[:,0],fm[:,1],fm[:,2],color=C["orig"],lw=1.8,label=f'原始({d["Tfm"]}帧)')
        if show_promp:ax3.plot(rp[:,0],rp[:,1],rp[:,2],color=C["promp"],lw=1.5,label=f'ProMP {d["rmse_p"]:.1f}mm')
        if show_ens and ensemble is not None:ax3.plot(ensemble[:,0],ensemble[:,1],ensemble[:,2],color=C["ens"],lw=1.2,alpha=0.7,ls=':',label='Ensemble')
        ax3.set_xlim(rx);ax3.set_ylim(ry);ax3.set_zlim(rz);ax3.set_title('3D');ax3.legend(fontsize=6)
        ax_r=self.fig.add_subplot(gs[0,2]);meths=['ProMP','DMP'];rmses=[d["rmse_p"],d["rmse_d"]];clrs=[C["promp"],C["dmp"]]
        bars=ax_r.bar(meths,rmses,color=clrs,alpha=0.8,width=0.4)
        for b,v in zip(bars,rmses):ax_r.text(b.get_x()+b.get_width()/2,b.get_height()+0.5,f'{v:.1f}mm',ha='center',fontsize=11,fontweight='bold')
        ax_r.set_title('RMSE');ax_r.axhline(y=5,color='green',ls='--',alpha=0.5);ax_r.grid(alpha=0.3,axis='y')
        ax_xy=self.fig.add_subplot(gs[1,0])
        if show_orig:ax_xy.plot(fm[:,0],fm[:,1],color=C["orig"],lw=0.8,alpha=0.6,label='原始')
        if show_promp:ax_xy.plot(rp[:,0],rp[:,1],color=C["promp"],lw=1.2,alpha=0.8,label='ProMP')
        ax_xy.set_xlabel('X');ax_xy.set_ylabel('Y');ax_xy.set_title('XY');ax_xy.legend(fontsize=6);ax_xy.grid(alpha=0.3)
        ax_xy.set_xlim(fm[:,0].min()-0.002,fm[:,0].max()+0.002);ax_xy.set_ylim(fm[:,1].min()-0.002,fm[:,1].max()+0.002)
        ax_s=self.fig.add_subplot(gs[1,1]);xv=np.diff(fm[:,0])/DT;nv=len(xv)
        if nv>30:
            nperseg=min(64,nv//4);noverlap=min(48,nv//5)
            if nperseg>2:f_spec,t_spec,Sxx=spectrogram(xv,fs=1/DT,nperseg=nperseg,noverlap=noverlap);ax_s.pcolormesh(t_spec,f_spec,10*np.log10(Sxx+1e-10),shading='gouraud',cmap='viridis',vmin=-40,vmax=10)
        ax_s.set_title('时频图');ax_s.set_xlabel('秒');ax_s.set_ylabel('Hz');ax_s.set_ylim(0.3,10)
        ax_xz=self.fig.add_subplot(gs[1,2])
        if show_orig:ax_xz.plot(fm[:,0],fm[:,2],color=C["orig"],lw=0.8,alpha=0.6,label='原始')
        if show_promp:
            ax_xz.plot(rp[:,0],rp[:,2],color=C["promp"],lw=1.2,alpha=0.8,label='ProMP')
            err=np.sqrt(np.sum((fm-rp)**2,axis=1))*1000;ax_xz.text(0.02,0.98,f'Max={err.max():.1f}mm',transform=ax_xz.transAxes,fontsize=8,va='top')
        ax_xz.set_xlabel('X');ax_xz.set_ylabel('Z');ax_xz.set_title('XZ');ax_xz.legend(fontsize=6);ax_xz.grid(alpha=0.3)
        ax_xz.set_xlim(fm[:,0].min()-0.002,fm[:,0].max()+0.002);ax_xz.set_ylim(fm[:,2].min()-0.002,fm[:,2].max()+0.002)
        self.canvas.draw_idle()
