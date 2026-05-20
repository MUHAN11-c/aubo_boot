"""后台工作线程"""
import os, sys, numpy as np
SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)
from PyQt5.QtCore import QThread, pyqtSignal
from scipy.signal import savgol_filter
from scipy.fft import fft, fftfreq
from scipy.interpolate import interp1d
DT=0.05; CARTESIAN_DIR=os.path.join(PKG_DIR,"resource","cartesian","right"); TOP5=[32,25,23,6,8]

def extract_forming(pos):
    z=pos[:,2];w=min(21,len(z)//10*2+1)
    if w<3:return pos,0,len(pos)
    zs=savgol_filter(z,w,3);lo=zs<np.median(zs)
    ch=np.diff(lo.astype(int));st=np.where(ch==1)[0]+1;en=np.where(ch==-1)[0]+1
    if lo[0]:st=np.concatenate([[0],st])
    if lo[-1]:en=np.concatenate([en,[len(lo)]])
    if len(st)==0:return pos,0,len(pos)
    b=np.argmax(en-st);fs,fe=st[b],en[b]
    if fe-fs<40:fs,fe=len(pos)//4,3*len(pos)//4
    return pos[fs:fe],fs,fe

class TrainWorker(QThread):
    finished=pyqtSignal(dict)
    def __init__(self,form_raw,nb_promp=20): super().__init__(); self.form_raw=form_raw; self.nb_promp=nb_promp
    def run(self):
        from latte_imitation.promp_learner import ProMP3D
        from latte_imitation.dmp_learner import DMP3D
        Tf=len(self.form_raw)
        p=ProMP3D(n_basis=self.nb_promp,sigma=0.03);p.learn(self.form_raw);rp=p.generate(T=Tf)
        rp_rmse=np.sqrt(np.mean(np.sum((self.form_raw-rp)**2,axis=1)))*1000
        d=DMP3D(n_basis=25,dt=0.01);d.learn(self.form_raw,tau=1.0);rd=d.generate(tau=1.0,T=Tf)
        rd_rmse=np.sqrt(np.mean(np.sum((self.form_raw-rd)**2,axis=1)))*1000
        self.finished.emit({"recon_p":rp,"rmse_p":rp_rmse,"recon_d":rd,"rmse_d":rd_rmse})

class CalibrateWorker(QThread):
    progress=pyqtSignal(str);finished=pyqtSignal(dict)
    def __init__(self,episodes="all"): super().__init__(); self.episodes=episodes
    def run(self):
        from latte_imitation.trajectory import CartesianTrajectory
        self.progress.emit("加载轨迹...")
        all_f=[];eps=range(40) if self.episodes=="all" else TOP5
        for ep in eps:
            path=os.path.join(CARTESIAN_DIR,f"episode_{ep:06d}.npz")
            if os.path.exists(path): all_f.append(extract_forming(CartesianTrajectory.load(path).positions)[0])
        self.progress.emit(f"分析{len(all_f)}条...")
        all_freqs,all_amps=[],[]
        for frm in all_f:
            xv=np.diff(frm[:,0])/DT;nv=len(xv)
            if nv>10:
                yf=np.abs(fft(xv-xv.mean()));xf_arr=fftfreq(nv,DT)
                pm=(xf_arr>0.5)&(xf_arr<9)
                if pm.sum()>0:all_freqs.extend(xf_arr[pm][np.argsort(yf[pm])[-3:]].tolist())
            all_amps.append(np.abs(xv).std()*1000)
        af=np.array(all_freqs) if all_freqs else np.array([2.4]);aa=np.array(all_amps) if all_amps else np.array([1.8])
        self.finished.emit({"freq_median":float(np.median(af)),"freq_iqr":[float(np.percentile(af,25)),float(np.percentile(af,75))],"amp_mean":float(aa.mean()),"amp_std":float(aa.std()),"n_episodes":len(all_f)})

class CVWorker(QThread):
    progress=pyqtSignal(int,str);finished=pyqtSignal(dict)
    def __init__(self,all_forming): super().__init__(); self.all_forming=all_forming
    def run(self):
        from latte_imitation.promp_learner import ProMP3D
        from latte_imitation.dmp_learner import DMP3D
        n=len(self.all_forming);pr,dr,er=[],[],[]
        for i in range(n):
            test=self.all_forming[i];train=[self.all_forming[j] for j in range(n) if j!=i]
            p=ProMP3D(n_basis=20,sigma=0.03);p.learn_multiple(train);gp=p.generate(T=len(test))
            rp=np.sqrt(np.mean(np.sum((test-gp)**2,axis=1)))*1000
            resampled=[]
            for seg in train:
                T=len(seg);to,tn=np.linspace(0,1,T),np.linspace(0,1,120)
                resampled.append(np.column_stack([interp1d(to,seg[:,d],kind='linear')(tn) for d in range(3)]))
            tm=np.median(np.stack(resampled,axis=0),axis=0)
            d=DMP3D(n_basis=25,dt=0.01);d.learn(tm,tau=1.0);gdr=d.generate(tau=1.0,T=120)
            Tt=len(test);to2,tn2=np.linspace(0,1,120),np.linspace(0,1,Tt)
            gd=np.column_stack([interp1d(to2,gdr[:,j],kind='linear')(tn2) for j in range(3)])
            rd=np.sqrt(np.mean(np.sum((test-gd)**2,axis=1)))*1000
            ea=np.column_stack([interp1d(to2,tm[:,j],kind='linear')(tn2) for j in range(3)])
            re=np.sqrt(np.mean(np.sum((test-ea)**2,axis=1)))*1000
            pr.append(rp);dr.append(rd);er.append(re)
            self.progress.emit(int((i+1)/n*100),f'{i+1}/{n}')
        self.finished.emit({"promp":np.array(pr),"dmp":np.array(dr),"ens":np.array(er),"n":n})
