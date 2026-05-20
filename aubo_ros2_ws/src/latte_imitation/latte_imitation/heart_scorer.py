"""心形轨迹质量评分 — 纯几何评估"""
import numpy as np
from scipy.signal import savgol_filter
from scipy.fft import fft, fftfreq
WS_X=(-0.87,0.87);WS_Y=(-0.87,0.87);WS_Z=(-0.85,1.10)

def _extract_forming_indices(z,dt=0.05):
    w=min(21,len(z)//10*2+1)
    if w<3:return 0,len(z)
    zs=savgol_filter(z,w,3);lo=zs<np.median(zs)
    ch=np.diff(lo.astype(int));st=np.where(ch==1)[0]+1;en=np.where(ch==-1)[0]+1
    if lo[0]:st=np.concatenate([[0],st])
    if lo[-1]:en=np.concatenate([en,[len(lo)]])
    if len(st)==0:return len(z)//4,3*len(z)//4
    b=np.argmax(en-st);fs,fe=st[b],en[b]
    if fe-fs<10:fs,fe=len(z)//4,3*len(z)//4
    return fs,fe

def score_heart_trajectory(trajectory,dt=0.05,verbose=False):
    T=len(trajectory)
    if T<20:return 0.0,{"error":"轨迹太短"}
    x,y,z=trajectory[:,0],trajectory[:,1],trajectory[:,2];scores={};details={}
    fs,fe=_extract_forming_indices(z,dt);x_form=x[fs:fe]
    x_mid=(x_form.max()+x_form.min())/2;x_asym_rms=np.sqrt(np.mean((x_form-x_mid)**2))
    sym_score=max(0,30*(1-x_asym_rms/0.030));scores["symmetry"]=sym_score;details["x_asym_rms_mm"]=round(x_asym_rms*1000,1)
    form_ratio=(fe-fs)/T
    if 0.05<=form_ratio<=0.60:form_score=25.0
    elif form_ratio<0.05:form_score=25*form_ratio/0.05
    else:form_score=25*(1-form_ratio)/0.40
    scores["forming_ratio"]=max(0,form_score);details["forming_ratio"]=round(form_ratio,3)
    x_vel=np.diff(x_form)/dt;nv=len(x_vel);peak_freq=0.0
    if nv>10:
        yf=np.abs(fft(x_vel-x_vel.mean()));xf_arr=fftfreq(nv,dt);pm=(xf_arr>0.3)&(xf_arr<12)
        if pm.sum()>0:peak_freq=xf_arr[pm][np.argmax(yf[pm])]
    if 0.8<=peak_freq<=8.0:freq_score=15.0
    elif peak_freq<0.8:freq_score=15*peak_freq/0.8
    else:freq_score=15*max(0,1-(peak_freq-8)/10)
    scores["wiggle_freq"]=max(0,freq_score);details["peak_freq_hz"]=round(peak_freq,2)
    x_amp_std=np.abs(x_form-x_form.mean()).std()*1000
    if 0.5<=x_amp_std<=15:amp_score=10.0
    elif x_amp_std<0.5:amp_score=10*x_amp_std/0.5
    else:amp_score=10*max(0,1-(x_amp_std-15)/30)
    scores["wiggle_amp"]=max(0,amp_score);details["x_amp_std_mm"]=round(x_amp_std,2)
    violations=0
    for i in range(T):
        if not(WS_X[0]<=x[i]<=WS_X[1]):violations+=1
        if not(WS_Y[0]<=y[i]<=WS_Y[1]):violations+=1
        if not(WS_Z[0]<=z[i]<=WS_Z[1]):violations+=1
    violation_ratio=violations/(T*3);ws_score=max(0,20*(1-violation_ratio))
    scores["workspace"]=ws_score;details["ws_violations"]=violations
    total=sum(scores.values());details["scores_breakdown"]={k:round(v,2) for k,v in scores.items()}
    return round(total,2),details

def score_against_reference(trajectory,reference,dt=0.05):
    from scipy.interpolate import interp1d
    T_ref=len(reference);T_traj=len(trajectory)
    t_ref=np.linspace(0,1,T_ref);t_traj=np.linspace(0,1,T_traj)
    ref_interp=np.column_stack([interp1d(t_ref,reference[:,d],kind='linear')(t_traj) for d in range(3)])
    rmse=np.sqrt(np.mean(np.sum((trajectory-ref_interp)**2,axis=1)))*1000
    score=max(0,min(100,100*(1-(rmse-5)/195)))
    return round(score,2),{"rmse_mm":round(rmse,2)}
