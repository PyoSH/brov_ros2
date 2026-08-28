# 로봇의 원시 상태 한 스냅샷. brov_base_node만 발행한다.
#
# 이 메시지는 "로봇이 지금 어떤 상태인가"만 담는다 — 목표도, 오차도, 적분도
# 없다. 그것들은 각각 DesiredState와 Observation의 몫이다.
#
# 프레임: 자세/위치는 호출자가 설정한 world frame(NED 또는 start-heading),
# 속도/각속도는 body frame. observation 단계가 Z-up/FLU 변환을 수행한다.

std_msgs/Header header

# 단조 증가. 소비자가 누락/중복을 감지하는 유일한 수단이다 — stamp만으로는
# 같은 샘플이 두 번 발행된 것과 새 샘플을 구분할 수 없다.
uint64 seq

geometry_msgs/Quaternion attitude        # world -> body, [w,x,y,z] 규약은 wxyz
geometry_msgs/Vector3 position           # world frame [m]
geometry_msgs/Vector3 linear_velocity    # body frame [m/s]
geometry_msgs/Vector3 angular_velocity   # body frame [rad/s]

# 속도의 출처. 논문은 DVL 직결을 쓰고 우리 기본은 EKF3다 — 어느 쪽으로
# 돌았는지가 결과 해석을 바꾸므로 wire에 싣는다.
string velocity_source                   # "mavlink_ekf" | "dvl" | "gazebo_truth"

# 신선도. 소비자는 이 값으로 stale을 판정한다(자기 시계와 stamp를 비교하는
# 것보다 안전하다 — 노드 간 시계가 다를 수 있다).
float32 attitude_age_s
float32 position_age_s

bool valid                               # false면 아래 reason에 이유
string reason
