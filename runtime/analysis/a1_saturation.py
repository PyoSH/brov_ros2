import sys, numpy as np, torch
from brov_base.diag_loop_delay import read_bag, _iter_messages
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from brov_base.vendor.params import load_brov2_yaml, thruster_pos_dir_ned
from brov_base.vendor.thruster import BROV2ThrusterModel, build_allocation_matrix
import os
_LIMIT = float(os.environ.get("A1_SECONDS", "0")) or None   # skip 이후 사용할 길이 [s]

SCALE = np.array([85., 85., 120., 26., 14., 22.]); AXN = ["surge","sway","heave","roll","pitch","yaw"]
cfg = load_brov2_yaml(); pos, dir_ = thruster_pos_dir_ned(cfg)
tm = BROV2ThrusterModel(num_envs=1, dt=0.04, device="cpu", pos=pos, dir=dir_, voltage=14.8)
B_pinv = torch.linalg.pinv(build_allocation_matrix(tm._pos, tm._dir)); lo, hi = tm.force_limits_n
def window(path):
    wr_t, wr_f, st_t, st_v, act_t, act_v = read_bag(path)
    t0 = act_t[act_v][0] + 5.0; t1 = min(wr_t[-1], st_t[-1])
    if _LIMIT: t1 = min(t1, t0 + _LIMIT)
    return wr_t, wr_f, t0, t1
def obs(path, t0, t1):
    it=_iter_messages(path); types=next(it); rows=[]
    for topic,data,stamp in it:
        if topic=="/brov/observation":
            m=deserialize_message(data,get_message(types[topic])); t=stamp*1e-9
            if t0<=t<=t1 and m.valid and len(m.data)==16: rows.append(list(m.data))
    return np.array(rows)
# 정책 A/B 에서는 둘 다 gain 1.0 이다 -- 라벨과 gain 을 인자로 받는다.
_L1 = sys.argv[5] if len(sys.argv) > 5 else "A(기준)"
_L2 = sys.argv[6] if len(sys.argv) > 6 else "B(비교)"
_G1 = float(sys.argv[7]) if len(sys.argv) > 7 else 1.0
_G2 = float(sys.argv[8]) if len(sys.argv) > 8 else 0.5
for label, path, gain in ((_L1, sys.argv[1], _G1), (_L2, sys.argv[2], _G2)):
    wr_t, wr_f, t0, t1 = window(path); m=(wr_t>=t0)&(wr_t<=t1); W=wr_f[m]
    print(f"\n=== {label}  ({m.sum()} 명령, {t1-t0:.0f}s) ===")
    print("(a) 행동 포화 |a|>=0.99  :", "  ".join(f"{AXN[i]} {100*np.mean(np.abs(W[:,i])>=0.99*SCALE[i]*gain):4.0f}%" for i in range(6)))
    print("    행동 평균 a=w/(scale*g):", "  ".join(f"{AXN[i]} {np.mean(W[:,i])/(SCALE[i]*gain):+.2f}" for i in range(6)))
    O = obs(path, t0, t1)
    if len(O):
        ve, om, zv, zq = O[:,4:7], O[:,7:10], O[:,10:13], O[:,13:16]
        print(f"(b) 관측 {len(O)}개: v_e 평균 {np.round(ve.mean(0),3)}  |v_e| RMS {np.round(np.sqrt((ve**2).mean(0)),3)}")
        print(f"    z_v 평균 {np.round(zv.mean(0),2)}  |z_v|>=4.9 비율 {np.round(100*np.mean(np.abs(zv)>=4.9,0),0)}%")
        print(f"    z_q 평균 {np.round(zq.mean(0),2)}  |z_q|>=4.9 비율 {np.round(100*np.mean(np.abs(zq)>=4.9,0),0)}%")
        print(f"    omega RMS {np.round(np.sqrt((om**2).mean(0)),2)} rad/s")
    thr = (B_pinv @ torch.tensor(W, dtype=torch.float32).T).T.numpy()
    clamp = np.mean((thr <= lo*0.99) | (thr >= hi*0.99), axis=0)
    print(f"(c) 추진기 클램프({lo:.0f}/{hi:.0f} N) 비율: " + " ".join(f"T{i+1} {100*c:3.0f}%" for i,c in enumerate(clamp)))
print("\n=== (d) 개루프 기준: A2 yaw / deadtime heave 의 1.8~2.6 Hz 대역 (명령에 2 Hz 가 없는데 응답에 있으면 기계적) ===")
def band(x, fs, lo_, hi_):
    X=np.fft.rfft(x-x.mean()); f=np.fft.rfftfreq(len(x),1/fs); mm=(f>=lo_)&(f<=hi_)
    return np.sqrt(2*np.sum(np.abs(X[mm])**2)/len(x)**2)
for label, path, ci, vi in (("A2 yaw", sys.argv[3], 5, 5), ("deadtime heave", sys.argv[4], 2, 2)):
    wr_t, wr_f, st_t, st_v, act_t, act_v = read_bag(path); t0=act_t[act_v][0]+5; t1=min(t0+25, wr_t[-1], st_t[-1])
    g=np.arange(t0,t1,0.02); f=np.interp(g,wr_t,wr_f[:,ci]); a=np.gradient(np.interp(g,st_t,st_v[:,vi]),g)
    print(f"  {label:15s} 명령 2Hz대역 {band(f,50,1.8,2.6):6.2f}  (1Hz대역 {band(f,50,0.8,1.2):6.2f})   응답 2Hz대역 {band(a,50,1.8,2.6):6.3f}  (1Hz대역 {band(a,50,0.8,1.2):6.3f})")
