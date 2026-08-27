"""Water Linked A50 DVL의 body 속도를 직접 읽는다 (json_v3 TCP 스트림).

왜 이게 필요한가
----------------
지금까지 기록한 속도는 전부 ArduSub EKF3 출력(``LOCAL_POSITION_NED``)이다.
DVL은 ``EK3_SRC1_VELXY``로 그 안에 융합돼 있을 뿐 직결 경로가 없었다. 그런데
논문이 쓰는 것은 **DVL body velocity 직결**이고, 항력 측정에서 속도가 조금만
과소 보고돼도 ``Xuu``가 제곱으로 틀어진다.

여기서는 EKF를 대체하지 않는다. **같은 시각의 두 값을 나란히 기록**해서 사후에
교차검증할 수 있게만 한다. 둘이 갈라지면 그 자체가 진단이다.

프레임 주의
-----------
``vx/vy/vz``는 DVL 자체 프레임이다. 장착 회전(A50의 ``Mounting rotation offset``)
이 0이면 vx가 전방이지만, 그 값은 장비 설정에 있고 여기서는 모른다. 그래서
**변환하지 않고 원시값 그대로 기록**한다. 주행 구간에서 EKF ``u``와 비교하면
부호·축 대응이 바로 드러난다.
"""

from __future__ import annotations

import json
import socket
import threading
import time


class DvlReader:
    """백그라운드 스레드로 최신 velocity 보고 하나를 유지한다.

    끊기거나 없어도 절대 예외를 올리지 않는다 — 측정 자체를 막으면 안 된다.
    """

    def __init__(self, host: str, port: int = 16171, *,
                 reconnect_s: float = 2.0) -> None:
        self.host = str(host)
        self.port = int(port)
        self.reconnect_s = float(reconnect_s)
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._latest_rx = 0.0
        self._connected = False
        self._error = ""
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------ 수명주기
    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="dvl_reader")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None

    # ------------------------------------------------------------ 조회
    def sample(self, *, max_age_s: float = 0.5) -> dict:
        """기록용 평탄한 dict. 값이 없거나 늦으면 전부 None이다."""
        with self._lock:
            latest, rx, connected, error = (
                self._latest, self._latest_rx, self._connected, self._error)
        age = time.monotonic() - rx if latest is not None else None
        fresh = latest is not None and age is not None and age <= max_age_s
        if not fresh:
            return {
                "dvl_vx": None, "dvl_vy": None, "dvl_vz": None,
                "dvl_valid": False, "dvl_fom": None, "dvl_altitude": None,
                "dvl_beams_valid": None, "dvl_age_s": age,
                "dvl_connected": connected, "dvl_error": error,
            }
        return {
            "dvl_vx": latest.get("vx"),
            "dvl_vy": latest.get("vy"),
            "dvl_vz": latest.get("vz"),
            "dvl_valid": bool(latest.get("velocity_valid", False)),
            "dvl_fom": latest.get("fom"),
            "dvl_altitude": latest.get("altitude"),
            "dvl_beams_valid": latest.get("beams_valid"),
            "dvl_age_s": age,
            "dvl_connected": True,
            "dvl_error": "",
        }

    @property
    def status(self) -> tuple[bool, str]:
        with self._lock:
            return self._connected, self._error

    # ------------------------------------------------------------ 내부
    @staticmethod
    def parse_line(line: bytes | str) -> dict | None:
        """velocity 보고 한 줄을 해석한다. 다른 type이면 None."""
        try:
            report = json.loads(line)
        except (ValueError, TypeError):
            return None
        if not isinstance(report, dict) or report.get("type") != "velocity":
            return None
        transducers = report.get("transducers")
        beams = None
        if isinstance(transducers, list):
            beams = sum(1 for t in transducers
                        if isinstance(t, dict) and t.get("beam_valid"))
        parsed = {
            "vx": report.get("vx"),
            "vy": report.get("vy"),
            "vz": report.get("vz"),
            "fom": report.get("fom"),
            "altitude": report.get("altitude"),
            # status 0 이 정상이다. velocity_valid 와 함께 봐야 한다.
            "velocity_valid": bool(report.get("velocity_valid", False))
            and int(report.get("status", 0) or 0) == 0,
            "beams_valid": beams,
        }
        if parsed["vx"] is None or parsed["vy"] is None or parsed["vz"] is None:
            return None
        return parsed

    def _run(self) -> None:
        buffer = b""
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.host, self.port),
                                              timeout=3.0) as connection:
                    connection.settimeout(1.0)
                    with self._lock:
                        self._connected, self._error = True, ""
                    buffer = b""
                    while not self._stop.is_set():
                        try:
                            chunk = connection.recv(8192)
                        except socket.timeout:
                            continue
                        if not chunk:
                            raise ConnectionError("스트림이 닫혔다")
                        buffer += chunk
                        # 한 줄이 비정상적으로 길면 버린다 (동기 손실 방어).
                        if len(buffer) > 1 << 20:
                            buffer = b""
                            continue
                        while b"\n" in buffer:
                            raw, buffer = buffer.split(b"\n", 1)
                            parsed = self.parse_line(raw)
                            if parsed is None:
                                continue
                            with self._lock:
                                self._latest = parsed
                                self._latest_rx = time.monotonic()
            except Exception as error:            # noqa: BLE001 — 절대 죽지 않는다
                with self._lock:
                    self._connected = False
                    self._error = f"{type(error).__name__}: {error}"
                self._stop.wait(self.reconnect_s)
