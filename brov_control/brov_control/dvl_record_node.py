#!/usr/bin/env python3
"""A50 DVL 속도를 **기록 전용**으로 발행한다.

왜 되먹임이 아니라 기록인가
===========================
논문 §5.2 는 DVL body velocity 를 직결로 쓴다. 우리 제어는 아직 ArduSub EKF3
출력을 쓰고, 그 편차가 2026-08-28 수조에서 드러났다 -- EKF 가 DVL 대비 12.9%
과소 보고했고 heave 축은 부호가 반대였다(상관 -0.722).

그래도 지금 승격하지 않는 이유는 둘이다.

**축·부호 변환이 미확정이다.** A50 의 Mounting rotation offset 은 장비 설정에
있고 코드는 모른다. 그래서 이 노드는 **변환하지 않고 원시값 그대로 낸다** --
가정을 데이터에 박으면 사후 검증이 불가능해진다. 주행 구간에서 EKF 와 나란히
놓고 보면 축 대응과 부호가 드러난다.

**위상 지연이 늘어난다.** DVL 은 5~15 Hz 이고 제어 루프는 25 Hz 다. 직결하면
2 Hz 에서 ZOH 만으로 약 -36도, 센서 지연까지 더하면 -108도 까지 붙는다. 지금
배포의 phase margin 이 이미 -24도 라(2026-08-31 측정) 그대로 넣으면 진동이
나빠진다. EKF 는 느린 DVL 을 빠른 IMU 로 메워 25 Hz 로 내보내므로 위상 면에서
유리하다 -- 대신 편향이 있다. 승격은 학습에 DVL 특성(``enable_dvl_realism``)을
넣은 뒤의 일이다.

EKF 융합을 밀어내지 않는지 확인할 것
====================================
A50 의 TCP 서버가 단일 클라이언트만 받는다면, 이 노드가 BlueOS 의 DVL
extension 을 밀어낼 수 있다. 그러면 EKF 는 IMU dead reckoning 으로 떨어지고
속도 추정이 조용히 나빠진다 -- **증상이 눈에 안 띄므로 반드시 확인해야 한다.**

확인 방법은 ``BrovState.ekf_velocity_variance`` 다(EKF_STATUS_REPORT 에서 온다).
이 노드를 붙이기 전후로 비교해서 눈에 띄게 오르면 융합이 끊긴 것이다::

    ros2 topic echo /brov/state --field ekf_velocity_variance    # 붙이기 전
    ros2 run brov_control dvl_record_node --ros-args -p dvl_host:=192.168.2.95
    ros2 topic echo /brov/state --field ekf_velocity_variance    # 붙인 후

이 노드는 split_stack 과 **별도로** 띄운다. 문제가 보이면 이것만 즉시 끄면
되고, 제어 경로는 건드리지 않는다.

발행
====
    /brov/dvl/sample    DvlSample   (원시 프레임, 변환 없음)
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from brov_interfaces.msg import DvlSample

from .dvl_reader import DvlReader


class DvlRecordNode(Node):
    """A50 을 읽어 그대로 발행한다. 제어에는 쓰이지 않는다."""

    def __init__(self) -> None:
        super().__init__("brov_dvl_record")
        self.declare_parameter("dvl_host", "192.168.2.95")
        self.declare_parameter("dvl_port", 16171)
        self.declare_parameter("publish_rate_hz", 25.0)
        # 이보다 오래된 표본은 무효로 본다. DVL 이 5~15 Hz 이므로
        # 0.5 s 면 여러 주기를 놓친 것이다.
        self.declare_parameter("max_age_s", 0.5)

        host = str(self.get_parameter("dvl_host").value)
        port = int(self.get_parameter("dvl_port").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._max_age_s = float(self.get_parameter("max_age_s").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._seq = 0
        self._reader = DvlReader(host, port)
        self._reader.start()
        self._pub = self.create_publisher(DvlSample, "/brov/dvl/sample", 20)
        # 2026-09-02 실기: 이 노드가 A50 에 붙은 78 s 뒤 BlueOS DVL extension 의
        # VISION_POSITION_DELTA 가 멈췄고, 5 s 뒤 EKF 가 CONST_POS_MODE 로 떨어져
        # LOCAL_POSITION_NED 가 끊겼다. 노드를 내려도 extension 은 스스로 회복하지
        # 않았다(25 분 뒤에도 멈춤). 정책 주행과 **같이 띄우지 말 것.**
        self.get_logger().warn(
            "DVL 기록기는 A50 의 TCP 슬롯을 차지해 BlueOS DVL extension 을 밀어낼 수 "
            "있다 -- 그러면 EKF 가 위치를 잃고 LOCAL_POSITION_NED 가 끊긴다. "
            "정책 주행 중에는 켜지 말 것. 끄고 나서 extension 이 안 돌아오면 "
            "BlueOS 에서 재시작할 것 (runtime/check_ekf.sh 로 확인).")
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"dvl_record_node 시작 — {host}:{port}, {rate:.0f} Hz 재발행. "
            "기록 전용이며 제어 경로에 들어가지 않는다. "
            "EKF 융합이 끊기지 않았는지 BrovState.ekf_velocity_variance 로 확인할 것"
        )

    def _tick(self) -> None:
        # DvlReader.sample() 은 평탄한 dict 를 준다. 값이 없거나 max_age_s 보다
        # 늦으면 전부 None 이고 dvl_valid=False 다 -- 우리는 그것을 숨기지 않고
        # 그대로 valid=False 로 내보낸다.
        snap = self._reader.sample(max_age_s=self._max_age_s)
        msg = DvlSample()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.seq = self._seq
        self._seq += 1
        msg.connected = bool(snap.get("dvl_connected", False))
        msg.reason = str(snap.get("dvl_error") or "")

        def _f(key, default=-1.0):
            value = snap.get(key)
            try:
                value = float(value)
            except (TypeError, ValueError):
                return default
            return value if math.isfinite(value) else default

        msg.velocity_raw = Vector3(x=_f("dvl_vx", 0.0), y=_f("dvl_vy", 0.0),
                                   z=_f("dvl_vz", 0.0))
        msg.fom = _f("dvl_fom")
        msg.altitude = _f("dvl_altitude")
        beams = snap.get("dvl_beams_valid")
        msg.beams_valid = int(beams) if isinstance(beams, int) else -1
        msg.valid = bool(snap.get("dvl_valid", False))
        if not msg.valid and not msg.reason:
            age = snap.get("dvl_age_s")
            msg.reason = ("표본 없음" if age is None
                          else f"velocity_valid=false 또는 stale ({age:.2f}s)")
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self._reader.stop()
        finally:
            super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DvlRecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
