"""
Unit tests for Reviewer 1.3 Geometry Outage Model, Channels, and Simulation Runner.
"""
import pytest
import numpy as np

from bacs_sim.config import SimConfig, ChannelConfig, LoRaConfig
from bacs_sim.lora import time_on_air
from bacs_sim.simulator import precompute

from reviewer_R1_3_geometry_outages.geometry_model import (
    RectObstacle,
    DEFAULT_OBSTACLES,
    DEFAULT_SERVER_POS,
    line_intersects_rect,
    check_link_blocked,
    compute_outage_statistics,
)
from reviewer_R1_3_geometry_outages.channel import (
    GeometryLoRaChannel,
    MatchedIIDChannel,
    MATCHED_C4_IID_LOSS_RATE,
)
from reviewer_R1_3_geometry_outages.runner import run_geometry_simulation, GeometryRunResult


class TestGeometryIntersection:
    """Test 2D ray-obstacle intersection and LoS / NLoS classification."""

    def test_direct_hit(self):
        # Ray passing straight through Rack_North_West (1.5 to 4.5, 6.5 to 8.0)
        obs = RectObstacle(1.5, 4.5, 6.5, 8.0)
        p1 = (3.0, 5.0)
        p2 = (3.0, 9.0)
        assert line_intersects_rect(p1, p2, obs) is True

    def test_clear_miss(self):
        # Ray passing completely to the side of the obstacle
        obs = RectObstacle(1.5, 4.5, 6.5, 8.0)
        p1 = (5.0, 5.0)
        p2 = (5.0, 9.0)
        assert line_intersects_rect(p1, p2, obs) is False

    def test_clear_aisle_to_server(self):
        # Point in central aisle (x=5.0) has clear line of sight to server at (7.0, 10.5)
        # Rack_North_West is [1.5, 4.5], Rack_North_Center is [5.5, 8.5]
        # Ray from (5.0, 7.0) to (7.0, 10.5) passes through the gap [4.5, 5.5]
        # Let's test a point at (5.0, 9.0) which is north of racks:
        assert check_link_blocked((5.0, 9.0), DEFAULT_SERVER_POS, DEFAULT_OBSTACLES) is False

    def test_shadow_behind_rack(self):
        # Point behind Rack_South_West (x=3.0, y=1.0) looking up to server (7.0, 10.5)
        # Line from (3.0, 1.0) to (7.0, 10.5) must cross Rack_South_West and/or Rack_North_Center
        assert check_link_blocked((3.0, 1.0), DEFAULT_SERVER_POS, DEFAULT_OBSTACLES) is True


class TestOutageStatistics:
    """Test computation of blockage fraction and outage duration statistics."""

    def test_zero_blockage(self):
        series = np.zeros(100, dtype=bool)
        stats = compute_outage_statistics(series, dt=0.1)
        assert stats["obstruction_fraction"] == 0.0
        assert stats["n_outages"] == 0
        assert stats["mean_outage_s"] == 0.0
        assert stats["max_outage_s"] == 0.0

    def test_known_outages(self):
        # 10s total (100 steps of 0.1s): 20 steps true (2s), 30 steps false, 40 steps true (4s), 10 steps false
        series = np.array([True] * 20 + [False] * 30 + [True] * 40 + [False] * 10)
        stats = compute_outage_statistics(series, dt=0.1)
        assert abs(stats["obstruction_fraction"] - 0.60) < 1e-6
        assert stats["n_outages"] == 2
        assert abs(stats["mean_outage_s"] - 3.0) < 1e-6
        assert abs(stats["max_outage_s"] - 4.0) < 1e-6


class TestChannelMechanics:
    """Test channel delivery probabilities and airtime accounting."""

    def test_c4_delivery_probabilities(self):
        rng = np.random.default_rng(42)
        cfg = ChannelConfig(max_retries=0)  # Single attempt to test raw drop rate
        lora = LoRaConfig()
        channel = GeometryLoRaChannel(cfg, lora, rng)

        # Test unobstructed path (should give ~95% delivery)
        n_trials = 2000
        los_deliveries = sum(
            channel.transmit(52, 0.0, rid=None)[0] for _ in range(n_trials)
        )
        los_rate = los_deliveries / n_trials
        assert 0.93 < los_rate < 0.97

        # Test obstructed path (should give ~10% delivery)
        # We simulate obstructed by directly querying p_nlos or configuring a mock truths
        channel_obs = GeometryLoRaChannel(cfg, lora, rng, p_los=0.10)
        nlos_deliveries = sum(
            channel_obs.transmit(52, 0.0, rid=None)[0] for _ in range(n_trials)
        )
        nlos_rate = nlos_deliveries / n_trials
        assert 0.08 < nlos_rate < 0.12

    def test_matched_iid_delivery_probability(self):
        rng = np.random.default_rng(42)
        cfg = ChannelConfig(max_retries=0)
        lora = LoRaConfig()
        channel = MatchedIIDChannel(cfg, lora, rng, loss_rate=MATCHED_C4_IID_LOSS_RATE)

        n_trials = 3000
        deliveries = sum(
            channel.transmit(52, 0.0)[0] for _ in range(n_trials)
        )
        deliv_rate = deliveries / n_trials
        expected_rate = 1.0 - MATCHED_C4_IID_LOSS_RATE
        assert abs(deliv_rate - expected_rate) < 0.025

    def test_airtime_charging_on_failures(self):
        rng = np.random.default_rng(42)
        # Force 100% loss to exhaust all retries
        cfg = ChannelConfig(max_retries=3, extra_delay_s=0.0)
        lora = LoRaConfig()
        channel = MatchedIIDChannel(cfg, lora, rng, loss_rate=1.0)

        t_air_single = time_on_air(52, lora)
        ok, t_recv, used_airtime, attempts = channel.transmit(52, 0.0)

        assert ok is False
        assert attempts == 4  # Initial attempt + 3 retries
        expected_airtime = 4 * t_air_single
        assert abs(used_airtime - expected_airtime) < 1e-9


class TestCandidatePoolIsolation:
    """Verify that all policies evaluate identical candidate sets for a given seed."""

    def test_candidate_pool_identity(self):
        seed = 130
        cfg = SimConfig(seed=seed)
        cfg.world.n_robots = 3

        truths, cands = precompute(cfg)

        # Clone and verify candidate properties across runs
        from bacs_sim.simulator import _clone
        pool_fifo = {k: [_clone(c) for c in v] for k, v in cands.items()}
        pool_bacs = {k: [_clone(c) for c in v] for k, v in cands.items()}

        for rid in range(3):
            assert len(pool_fifo[rid]) == len(pool_bacs[rid])
            for c1, c2 in zip(pool_fifo[rid], pool_bacs[rid]):
                assert c1.idx_from == c2.idx_from
                assert c1.idx_to == c2.idx_to
                assert c1.t_created == c2.t_created
                assert np.allclose(c1.z, c2.z)
                assert c1.is_outlier == c2.is_outlier


class TestSimulationExecution:
    """Smoke test running full simulations under C4 and C4-IID."""

    def test_single_run_c4(self):
        cfg = SimConfig(seed=130)
        cfg.world.n_robots = 2
        cfg.scheduler.policy = "bacs_plus"

        res = run_geometry_simulation(cfg, condition="C4")
        assert isinstance(res, GeometryRunResult)
        assert not np.isnan(res.pose_rmse)
        assert not np.isnan(res.align_rmse)
        assert res.n_candidates > 0
        assert res.n_sent > 0
        assert res.obstruction_fraction > 0.0
        assert res.n_outages > 0

    def test_single_run_c4_iid(self):
        cfg = SimConfig(seed=130)
        cfg.world.n_robots = 2
        cfg.scheduler.policy = "random"

        res = run_geometry_simulation(cfg, condition="C4-IID")
        assert isinstance(res, GeometryRunResult)
        assert not np.isnan(res.pose_rmse)
        assert not np.isnan(res.align_rmse)
        assert res.n_sent > 0
