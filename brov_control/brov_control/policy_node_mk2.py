#!/usr/bin/env python3
"""MK2-only policy node with a metadata-bound explicit T6 contract."""

from __future__ import annotations

import rclpy

from .policy_contract import MK2_ACTION_CONTRACT
from .policy_node import PolicyNode


class PolicyNodeMk2(PolicyNode):
    """Reject legacy policies and apply FLU/Z-up -> SNAME/FRD before B+."""

    _POLICY_ACTION_CONTRACT = MK2_ACTION_CONTRACT


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyNodeMk2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
