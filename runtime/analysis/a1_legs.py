import sys, numpy as np
from brov_base.diag_loop_delay import read_bag, _iter_messages
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def band(x, fs, lo, hi):
    X=np.fft.rfft(x-x.mean()); f=np.fft.rfftfreq(len(x),1/fs); m=(f>=lo)&(f<=hi); return np.sqrt(2*np.sum(np.abs(X[m])**2)/len(x)**2)
for label, path in (("gain1.0", sys.argv[1]), ("gain0.5", sys.argv[2])):
    wr_t, wr_f, st_t, st_v, act_t, act_v = read_bag(path); t0=act_t[act_v][0]+5; t1=min(wr_t[-1], st_t[-1])
    it=_iter_messages(path); types=next(it); wp=[]; wpt=[]; vd=[]
    for topic,data,stamp in it:
        if topic=="/brov/desired":
            m=deserialize_message(data,get_message(types[topic])); t=stamp*1e-9
            if t0<=t<=t1: wpt.append(t); wp.append(m.waypoint_index); vd.append([m.velocity_body.x,m.velocity_body.y,m.velocity_body.z])
    wp=np.array(wp); vd=np.array(vd); switches=int(np.sum(np.diff(wp)!=0))
    m=(st_t>=t0)&(st_t<=t1); V=st_v[m]
    g=np.arange(t0,t1,0.02)
    print(f"\n=== {label} ({t1-t0:.0f}s) ===")
    print(f"  waypoint 전환 {switches}회 -> 다리당 평균 {(t1-t0)/max(switches,1):.1f}s  (1.0 m 를 0.25 m/s 로 직진하면 4 s)")
    print(f"  실제 body 속도  surge 평균 {V[:,0].mean():+.3f} m/s (RMS {np.sqrt((V[:,0]**2).mean()):.3f})   sway RMS {np.sqrt((V[:,1]**2).mean()):.3f}   heave RMS {np.sqrt((V[:,2]**2).mean()):.3f}")
    print(f"  desired v_d_b 평균 {np.round(vd.mean(0),3)}  |v_d| 평균 {np.linalg.norm(vd,axis=1).mean():.3f} m/s")
    for name, ci, vi, unit in (("roll",3,3,"N*m"),("pitch",4,4,"N*m"),("yaw",5,5,"N*m")):
        f=np.interp(g,wr_t,wr_f[:,ci]); a=np.gradient(np.interp(g,st_t,st_v[:,vi]),g)
        print(f"  {name:5s} 명령 2Hz대역 {band(f,50,1.8,2.6):5.2f} {unit} (평균 {f.mean():+5.2f}, 전체RMS {f.std():5.2f})   각가속도 2Hz대역 {band(a,50,1.8,2.6):5.2f} rad/s²")
