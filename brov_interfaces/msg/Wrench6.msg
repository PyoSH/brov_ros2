# 정책/컨트롤러 -> base 로 가는 명령. SNAME/FRD body frame.
#
# 절단면을 PWM이 아니라 wrench로 둔 이유:
#   base   가 액추에이터 고유 지식을 독점한다 — 할당행렬, T200 추력 테이블,
#          전압, deadband, 포화, slew 한계
#   policy 가 아티팩트 고유 지식을 독점한다 — 계약, wrench_scale, T6 부호, clamp
# 경계가 물리량이라 테스트할 수 있고, model_based_controller가 정책을
# 노드 교체 하나로 대체할 수 있다(이미 wrench를 낸다).

std_msgs/Header header
uint64 seq

geometry_msgs/Vector3 force              # [N]  surge, sway, heave
geometry_msgs/Vector3 torque             # [Nm] roll, pitch, yaw

# 생성자 식별. base가 로그와 진단에 남긴다.
string source                            # "policy" | "model_based" | "diag"
