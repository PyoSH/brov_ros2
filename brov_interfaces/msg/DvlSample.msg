# Water Linked A50 DVL 의 속도 보고 한 샘플. brov_dvl_record_node 만 발행한다.
#
# **되먹임 경로가 아니다.** 논문 §5.2 는 DVL body velocity 를 직결로 쓰지만,
# 우리 제어는 아직 ArduSub EKF3 출력을 쓴다. 이 토픽은 **같은 시각의 두 값을
# 나란히 남겨** 사후 교차검증하기 위한 것이다. 승격하려면 두 가지가 먼저다:
#   1. 장착 회전(A50 의 Mounting rotation offset)에 따른 축·부호 변환 확정
#      -- 2026-08-28 수조에서 EKF heave 와 DVL vz 의 상관이 -0.722 로 부호가
#      반대였다. 그 값은 장비 설정에 있고 코드는 모른다.
#   2. DVL 특성(5~15 Hz, 지연 0~150 ms)으로 학습 -- 지금 정책은 그것 없이
#      학습됐고, 느린 센서를 되먹임에 넣으면 위상 지연이 늘어 진동이 나빠진다.

std_msgs/Header header
uint64 seq

# **DVL 자체 프레임의 원시값이다.** 변환하지 않는다 -- 장착 회전을 모르는 채
# 변환하면 그 가정이 데이터에 박혀 사후 검증이 불가능해진다. 주행 구간에서
# EKF 와 비교하면 축 대응과 부호가 드러난다.
geometry_msgs/Vector3 velocity_raw       # [m/s] DVL 프레임

float32 fom                              # figure of merit [m/s], 작을수록 좋음
float32 altitude                         # 해저까지 거리 [m], 음수면 무효
int8 beams_valid                         # 유효 beam 수 (transducers 에서 셈), -1 이면 미보고

# velocity_valid 와 status==0 을 함께 본 결과. 둘 중 하나라도 어긋나면 false.
bool valid

# TCP 스트림 연결 상태. false 면 velocity 필드는 마지막 값이거나 0 이다.
bool connected
string reason
