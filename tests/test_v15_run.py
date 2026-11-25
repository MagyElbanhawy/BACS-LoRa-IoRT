from bacs_sim.config import SimConfig
from bacs_sim import simulator


def test_v15_routes_only_bacs_through_two_pass_selection(monkeypatch):
    config = SimConfig(seed=70)
    config.world.session_s = 60
    config.world.payload_jitter_bytes = 0
    config.world.obs_prob = 1.0
    config.world.obs_radius = 20
    config.scheduler.policy = "bacs_plus"
    config.scheduler.trust_gate = .05
    config.trust.gamma = .01
    observations = simulator.precompute(config)
    actual = simulator.select_window
    calls = []

    def inspect(pool, **options):
        calls.append(options)
        assert all(c.generated_s <= options["now_s"] and c.payload_bytes == 52 for c in pool)
        assert options["freshness_rate"] == options["gamma"] == .01
        assert options["now_s"] == 60
        assert options["bacs_plus"]
        return actual(pool, **options)

    monkeypatch.setattr(simulator, "select_window", inspect)
    simulator.run(config, observations, execution="v15")
    assert len(calls) == config.world.n_robots
    calls.clear()
    config.scheduler.policy = "random"
    simulator.run(config, observations, execution="v15")
    assert not calls
