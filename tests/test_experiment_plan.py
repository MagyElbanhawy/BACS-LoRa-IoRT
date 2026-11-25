import argparse
from pathlib import Path

import pytest

from ros2_ws.src.bacs_scheduler.bacs_scheduler.experiment_plan import (
    POLICIES, block_plan, expected_policy, policy_order)
from ros2_ws.src.bacs_scheduler.bacs_scheduler.hardware_session import commands, ros_args, validate


def namespace(**changes):
    values = dict(
        role="robot", session="HWS-BACS-001", run=1, policy="FIFO", duration=600.0,
        start_at=None, serial_port="/dev/ttyUSB0", baud=115200, log_dir="~/bacs_hw_logs",
        ros_domain_id=42, max_clock_offset_ms=1.0, pilot=False, network_id=18,
        power_dbm=14, robot="limo01", address=1, scan_topic=None, local_frame=None,
        base_frame=None, random_seed=7, server_address=100, start_poses=None,
        duty_cycle=0.01, duty_window_s=60.0, trust_threshold=0.5,
        observability_weight=0.30, observability_reference=6, defer_half_life_s=60.0,
        max_queue_age_s=600.0, keyframe_period_s=2.0, min_motion_m=0.05,
        min_rotation_deg=5.0, sectors=60, max_range=10.0, shortlist=3,
        place_threshold=0.12, min_inlier_ratio=0.8, max_rmse=0.05, min_points=60,
        trust_rmse_ref=0.05, info_sigma_ref=0.05, extra_topic=[], dry_run=True,
    )
    values.update(changes)
    return argparse.Namespace(**values)


def test_four_policy_order_is_balanced_over_four_blocks() -> None:
    orders = [policy_order(block) for block in range(1, 5)]
    assert orders[0] == POLICIES
    assert orders[1] == ("BACS", "BACS+", "RANDOM", "FIFO")
    for position in range(4):
        assert {order[position] for order in orders} == set(POLICIES)


def test_block_plan_has_unique_numbered_sessions_and_600_seconds() -> None:
    rows = [row for block in range(1, 5) for row in block_plan(block)]
    assert len({row.session for row in rows}) == 16
    assert [row.session for row in rows[:4]] == [
        "HWS-BACS-001", "HWS-BACS-002", "HWS-BACS-003", "HWS-BACS-004"
    ]
    assert all(row.duration_s == 600 for row in rows)
    assert [row.run for row in rows[4:8]] == [2, 2, 2, 2]


def test_session_validation_prevents_policy_mislabeling() -> None:
    validate(namespace())
    with pytest.raises(ValueError, match="block/run 1 policy FIFO"):
        validate(namespace(policy="RANDOM"))
    with pytest.raises(ValueError, match="block/run 2 policy BACS"):
        validate(namespace(session="HWS-BACS-005", run=1, policy="BACS"))
    with pytest.raises(ValueError, match="six measured"):
        validate(namespace(role="server", robot=None, address=None, start_poses=None))
    validate(namespace(role="server", robot=None, address=None, start_poses=[0, 0, 0, 0, 1, 0]))


def test_numbered_session_recovers_policy_from_balanced_plan() -> None:
    assert expected_policy("HWS-BACS-001") == (1, "FIFO")
    assert expected_policy("HWS-BACS-005") == (2, "BACS")
    assert expected_policy("HWS-BACS-008") == (2, "FIFO")
    assert expected_policy("HWS-BACS-012") == (3, "BACS")
    with pytest.raises(ValueError, match="form HWS-BACS-NNN"):
        expected_policy("HWS-101-FIFO")


def test_robot_commands_run_real_candidates_then_policy_sender() -> None:
    args = namespace()
    args.acquisition_start_ns = 123456789
    planned = commands(args, Path("/tmp/run"))
    assert [name for name, _ in planned] == ["candidate", "sender"]
    assert "bacs_candidates" in planned[0][1]
    assert "bacs_sender" in planned[1][1]
    assert "policy:=FIFO" in planned[1][1]
    assert "random_seed:=7" in planned[1][1]
    assert "duty_cycle:=0.01" in planned[1][1]
    assert "place_threshold:=0.12" in planned[0][1]
    assert "start_at_ns:=123456789" in planned[0][1]


def test_server_commands_record_bag_fusion_and_real_receive_side() -> None:
    args = namespace(role="server", robot=None, address=None, start_poses=[0, 0, 0, 0, 1, 0],
                     extra_topic=["/camera/color/image_raw/compressed"])
    planned = commands(args, Path("/tmp/run"))
    assert [name for name, _ in planned] == ["bag", "fusion", "receiver"]
    assert "/vicon/limo01/pose" in planned[0][1]
    assert "/camera/color/image_raw/compressed" in planned[0][1]
    assert "bacs_fusion" in planned[1][1] and "bacs_receiver" in planned[2][1]


def test_ros_parameter_lists_are_single_yaml_arguments() -> None:
    rendered = ros_args({"robots": ["limo01", "limo02"], "run": 2})
    assert rendered == ["--ros-args", "-p", "robots:=[limo01,limo02]", "-p", "run:=2"]
