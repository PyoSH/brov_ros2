"""Contracts for the operator-facing demo orchestrator."""

from __future__ import annotations

import inspect
import math
from pathlib import Path

import pytest

from brov_bringup.demo_orchestrator_node import (
    case_a_points,
    DemoOrchestratorNode,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_case_a_generator_allows_a_short_bottom_entry() -> None:
    first, second = case_a_points(
        (1.70, 0.75, 0.175618),
        (0.35, 0.30, 0.20),
        (3.65, 1.40, 0.90),
        0.20,
        0.30,
    )

    assert first == pytest.approx((1.70, 0.75, 0.20))
    assert second == pytest.approx((1.90, 0.75, 0.20))
    assert math.dist((1.70, 0.75, 0.175618), first) < 0.03


def test_case_a_generator_moves_toward_pool_centre() -> None:
    first, second = case_a_points(
        (2.70, 0.75, 0.30),
        (0.35, 0.30, 0.20),
        (3.65, 1.40, 0.90),
        0.20,
        0.30,
    )

    assert first == pytest.approx((2.70, 0.75, 0.30))
    assert second == pytest.approx((2.50, 0.75, 0.30))


def test_case_a_generator_rejects_a_distant_unsafe_start() -> None:
    with pytest.raises(ValueError, match="nearest safe first waypoint"):
        case_a_points(
            (1.70, -0.50, 0.175),
            (0.35, 0.30, 0.20),
            (3.65, 1.40, 0.90),
            0.20,
            0.30,
        )


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_case_a_generator_rejects_nonfinite_pose(bad: float) -> None:
    with pytest.raises(ValueError, match="must be finite"):
        case_a_points(
            (bad, 0.75, 0.30),
            (0.35, 0.30, 0.20),
            (3.65, 1.40, 0.90),
            0.20,
            0.30,
        )


def test_orchestrator_exposes_only_three_operator_services() -> None:
    constructor = inspect.getsource(DemoOrchestratorNode.__init__)

    for service in (
        "/brov/demo/prepare",
        "/brov/demo/start",
        "/brov/demo/stop",
    ):
        assert service in constructor
    assert "/brov/estop" not in constructor


def test_prepare_approves_full_se3_pool_initialization() -> None:
    ensure_localized = inspect.getsource(
        DemoOrchestratorNode._ensure_localized
    )

    assert 'self._call("confirm_neutral"' in ensure_localized
    assert "request = InitializePool.Request()" in ensure_localized
    assert "request.min_samples = 0" in ensure_localized
    assert '"initialize",' in ensure_localized
    assert "require_success=False" in ensure_localized
    assert '"residual gate left"' in ensure_localized
    assert '"INITIALIZING"' in ensure_localized
    assert "state=INITIALIZED(2)" in ensure_localized


def test_start_and_stop_reuse_authoritative_services_in_safe_order() -> None:
    start = inspect.getsource(DemoOrchestratorNode._start_impl)
    stop = inspect.getsource(DemoOrchestratorNode._stop_impl)
    cleanup = inspect.getsource(
        DemoOrchestratorNode._cleanup_after_start_failure
    )

    assert start.index('self._call("arm"') < start.index(
        'self._call("start"'
    )
    assert start.index('self._call("arm"') < start.index("start_mark =")
    assert start.index("start_mark =") < start.index('self._call("start"')
    assert '"first post-START controller PWM"' in start
    assert 'sequence = ["stop"]' in stop
    assert 'sequence.append("disarm")' in stop
    assert '"stop",' in cleanup
    assert '"disarm",' in cleanup


def test_reprepare_reuses_the_immutable_committed_path() -> None:
    prepare = inspect.getsource(DemoOrchestratorNode._prepare_impl)

    reuse = prepare.index("if self._active_path is not None:")
    generate = prepare.index("desired_path = None")
    assert reuse < generate
    assert 'self._call("prepare"' in prepare[reuse:generate]
    assert "reused committed pool path=" in prepare[reuse:generate]


def test_package_exports_orchestrator_console_script() -> None:
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert "demo_orchestrator_node = " in setup_source
    assert "brov_bringup.demo_orchestrator_node:main" in setup_source
