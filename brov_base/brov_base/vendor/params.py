"""BROV2 parameter loader backed by the package's ``brov2_heavy.yaml``."""
from __future__ import annotations
import os

import yaml

_BROV2_YAML_PATH = os.path.join(os.path.dirname(__file__), "brov2_heavy.yaml")


def load_brov2_yaml(yaml_path: str | os.PathLike[str] = _BROV2_YAML_PATH) -> dict:
    """brov2_heavy.yaml 전체를 읽어서 dict로 반환한다."""
    with open(yaml_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def coBM_vector_ned(params: dict) -> tuple[float, float, float]:
    """params['coBM'](Z-up body frame, COM 기준)을 SNAME/NED body frame
    (X=전방,Y=우현,Z=하방)으로 변환해 반환한다.
    """
    v = params["coBM"]
    return (v[0], -v[1], -v[2])  # Z-up -> SNAME/NED (T3, self-inverse)


def thruster_pos_dir_ned(params: dict) -> tuple[list, list]:
    """params['thrusters']['list']의 position/axis(Z-up body frame, USD 정본의 미러)를
    SNAME/NED body frame(X=전방,Y=우현,Z=하방)으로 변환해 (pos, dir) 리스트로 반환한다.

    리스트 순서가 곧 T1~T8 순서 — BROV2ThrusterModel._POS/_DIR과 동일 인덱싱.
    """
    pos, dir_ = [], []
    for t in params["thrusters"]["list"]:
        px, py, pz = t["position"]
        ax, ay, az = t["axis"]
        pos.append([px, -py, -pz])   # Z-up -> SNAME/NED (T3, self-inverse)
        dir_.append([ax, -ay, -az])
    return pos, dir_
