import sys, numpy as np, math
from brov_base.diag_loop_delay import _iter_messages
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
POOL_X, POOL_Y, POOL_Z = 3.30, 1.10, 0.70   # 안전영역 크기
def load(path):
    it=_iter_messages(path); types=next(it); t=[];P=[];Q=[];V=[];W=[];at=[];av=[]
    for topic,data,stamp in it:
        if topic=="/brov/state":
            m=deserialize_message(data,get_message(types[topic]))
            t.append(stamp*1e-9); P.append([m.position.x,m.position.y,m.position.z])
            Q.append([m.attitude.w,m.attitude.x,m.attitude.y,m.attitude.z])
            V.append([m.linear_velocity.x,m.linear_velocity.y,m.linear_velocity.z])
            W.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        elif topic=="/brov/control_active":
            at.append(stamp*1e-9); av.append(deserialize_message(data,get_message(types[topic])).data)
    t=np.array(t);P=np.array(P);Q=np.array(Q);V=np.array(V);W=np.array(W)
    at=np.array(at);av=np.array(av,bool); t0=at[av][0]
    m=t>=t0; return t[m]-t0, P[m]-P[m][0], Q[m], V[m], W[m]
L1 = sys.argv[3] if len(sys.argv) > 3 else "A"
L2 = sys.argv[4] if len(sys.argv) > 4 else "B"
for label, path in ((L1, sys.argv[1]), (L2, sys.argv[2])):
    t,P,Q,V,W = load(path)
    yaw=np.unwrap(np.array([math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)) for q in Q]))
    print(f"\n=== {label}  {t[-1]:.0f}s ===")
    print("  구간    yaw율(중앙)  |ω|RMS   위치이동/구간   누적경로   수조밖?")
    for a in range(0, int(t[-1]), 15):
        m=(t>=a)&(t<a+15)
        if m.sum()<20: continue
        dyaw=np.degrees(yaw[m][-1]-yaw[m][0])/15
        disp=np.linalg.norm(P[m][-1]-P[m][0])
        path_len=np.sum(np.linalg.norm(np.diff(P[m],axis=0),axis=1))
        out = "x" if (P[m][:,0].ptp()>POOL_X or P[m][:,1].ptp()>POOL_Y) else " "
        print(f"  {a:2d}-{a+15:2d}s  {dyaw:+7.0f}°/s  {np.sqrt((W[m]**2).sum(1).mean()):5.2f}  "
              f"{disp:8.2f} m     {path_len:6.1f} m    {out}")
    print(f"  전체: 위치 범위 x {P[:,0].ptp():.2f}  y {P[:,1].ptp():.2f} m   (수조 안전영역 {POOL_X}×{POOL_Y} m)")
    print(f"        yaw 순증 {np.degrees(yaw[-1]-yaw[0]):+.0f}°,  |ω_yaw| RMS {np.sqrt((W[:,2]**2).mean()):.2f} rad/s")
