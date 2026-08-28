# 정책 입력 16-D. brov_observation_node만 발행한다.
#
#   [0:4]   q_e   자세 오차 쿼터니언 conj(q_d) x q   [w,x,y,z]
#   [4:7]   v_e_b body frame 속도 오차 v_b - v_d_b
#   [7:10]  omega_b
#   [10:13] z_v   v_e_b 적분
#   [13:16] z_q   q_e 벡터부 적분
#
# 적분 상태는 **이 노드가 단독 소유**한다. 여러 노드가 각자 적분하면 주기가
# 갈라져 드리프트한다.

std_msgs/Header header
uint64 seq

float32[16] data

# 관측 계약 이름. 정책 아티팩트의 metadata와 **문자열 동일**해야 한다.
# 한 프로세스였을 때는 암묵적이었지만, 분리한 뒤에는 wire에서 강제해야 한다.
string contract                          # 예: "brov_velocity_observation_v2"

# 이 관측을 만든 상태/목표의 seq. 정책이 어느 입력에서 나왔는지 역추적한다.
uint64 state_seq
uint64 desired_seq

# 적분 시각 간격 [s]. 정책 주기와 다르면 학습 계약 위반이므로 소비자가 검사한다.
float32 integration_dt_s

bool valid
string reason
