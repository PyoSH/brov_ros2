# 유도(guidance)가 만든 목표. brov_guidance_node만 발행한다.
#
# step_3에서 LOSGuidance가 NBV/커버리지 모듈로 교체되는 지점이 정확히 여기다.
# 그때 바꾸는 것은 이 메시지를 발행하는 노드 하나뿐이고, 관측/정책은 무변경이다.

std_msgs/Header header
uint64 seq

# body frame 목표 속도 [m/s]. 학습 계약상 ||v_d_b|| = cruise_speed 로 정확히
# 보존돼야 한다 — 정책이 학습한 명령 크기는 그 값 하나뿐이다.
geometry_msgs/Vector3 velocity_body
geometry_msgs/Quaternion attitude        # 목표 자세 q_d

uint32 waypoint_index
bool mission_complete
