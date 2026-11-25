from math import exp, log

import pytest

from ros2_ws.src.bacs_scheduler.bacs_scheduler.scheduler import (
    Constraint,
    DutyCycleBudget,
    Scheduler,
    lora_airtime_s,
)
from ros2_ws.src.bacs_scheduler.bacs_scheduler import scheduler_logging
from ros2_ws.src.bacs_scheduler.bacs_scheduler import transport


def item(sequence: int, trust: float = 1.0, info: float = 1.0, pair_count: int = 0) -> Constraint:
    return Constraint(sequence, 0, 52, trust, info, "a", "b", pair_count)


def test_lora_airtime_is_positive_and_repeatable() -> None:
    # SF7 / BW125 / CR4/5 / preamble 8 / explicit header / CRC.
    assert lora_airtime_s(52) == pytest.approx(0.102656)
    # LoRa payload symbols are quantized, so adjacent byte sizes may share airtime.
    assert lora_airtime_s(56) > lora_airtime_s(52)


def test_one_percent_duty_cycle_and_window() -> None:
    budget = DutyCycleBudget(window_s=60, duty_cycle=.01)
    budget.record(0, .5)
    assert not budget.can_send(0, .11)
    assert budget.can_send(61, .6)


def test_fifo_preserves_generation_order() -> None:
    selected = Scheduler("FIFO", DutyCycleBudget()).select([item(2), item(1)], 0)
    assert [x.sequence for x in selected] == [1, 2]


def test_trust_gating() -> None:
    assert Scheduler("BACS", DutyCycleBudget(), trust_threshold=.5).select([item(1, .49), item(2, .5)], 0) == [item(2, .5)]


def test_bacs_ranking_and_observability() -> None:
    bacs = Scheduler("BACS", DutyCycleBudget())
    assert bacs.select([item(1, info=.1), item(2, info=.9)], 0)[0].sequence == 2
    assert item(1, pair_count=0).observability(6) == pytest.approx(1)
    assert item(1, pair_count=6).observability(6) == pytest.approx(exp(-1))


def test_temporal_decay_parameter() -> None:
    scheduler = Scheduler("BACS+", DutyCycleBudget(), defer_half_life_s=10)
    assert scheduler.gamma_defer == pytest.approx(log(2) / 10)


# ---------------------------------------------------------------------------
# RANDOM policy
# ---------------------------------------------------------------------------
def test_random_policy_is_supported_and_budget_respected() -> None:
    budget = DutyCycleBudget(window_s=60, duty_cycle=.01)
    sched = Scheduler("RANDOM", budget, random_seed=42)
    selected = sched.select([item(i) for i in range(1, 6)], 0)
    # 5 items of 52 B each at SF7/125k/4-5 -> ~92 ms airtime; 1% of 60 s = 600 ms
    # capacity, so all 5 fit.
    assert len(selected) == 5
    # RANDOM must not return them in pure ascending sequence order (extremely
    # unlikely for a real shuffle of 5 distinct values with a fixed seed).
    assert sorted(x.sequence for x in selected) == [1, 2, 3, 4, 5]


def test_random_policy_is_reproducible_across_runs() -> None:
    items = [item(i) for i in range(1, 11)]
    s1 = Scheduler("RANDOM", DutyCycleBudget(), random_seed=7).select(items, 0)
    s2 = Scheduler("RANDOM", DutyCycleBudget(), random_seed=7).select(items, 0)
    assert [x.sequence for x in s1] == [x.sequence for x in s2]


def test_random_policy_rejects_unknown_policies() -> None:
    with pytest.raises(ValueError):
        Scheduler("not-a-policy", DutyCycleBudget())


# ---------------------------------------------------------------------------
# Logging schema - explicit scheduler timestamps close the transparency gap.
# ---------------------------------------------------------------------------
def test_scheduler_logging_schema_includes_transparency_gap_fields() -> None:
    fields = scheduler_logging.SCHEDULER_FIELDS
    # The three fields added to close the historical transparency gap.
    for required in (
        "t_scheduler_enqueue_ns",
        "t_scheduler_dequeue_ns",
        "t_transport_handoff_ns",
    ):
        assert required in fields, f"missing transparency-gap field: {required}"


# ---------------------------------------------------------------------------
# T,seq,hex serial sender (replaces AT+SEND)
# ---------------------------------------------------------------------------
def test_t_seq_hex_sender_requires_remote_ack_and_logs_timing(tmp_path):
    port = transport._BytesPort()
    # TX_DONE is local evidence only; ACK_RX is the remote delivery proof.
    port.feed("TX_DONE,12345,7,102700")
    port.feed("ACK_RX,12380,7,-67,8.25")

    sender = transport.TSeqHexSerialSender(port=port, session_dir=tmp_path)
    ack = sender.transmit(seq=7, payload=b"7" * 48)
    sender.close()

    assert ack is not None
    assert ack.direction == "MR_DELIVERY_ACK"
    assert ack.seq == 7
    assert ack.device_uptime_ms == 12380
    assert ack.rssi_dbm == "-67"
    assert ack.snr_db == "8.25"

    # The host -> MR line must be the T,seq,hex format (NOT AT+SEND).
    sent = bytes(port.tx).decode("ascii")
    assert sent.startswith("T,7,"), sent
    assert sent.endswith("\n")

    # The serial log distinguishes local TX completion from remote delivery.
    log_text = (tmp_path / "serial_log.csv").read_text()
    assert "MR_TX_DONE" in log_text
    assert "MR_DELIVERY_ACK" in log_text

    # No false cross-clock offset is computed. Host monotonic RTT and the
    # board-reported TX duration are recorded instead.
    timing_text = (tmp_path / "timing_log.csv").read_text()
    assert "DELIVERED" in timing_text
    assert "102700" in timing_text
    assert not (tmp_path / "clock_offsets.txt").exists()


def test_t_seq_hex_sender_does_not_emit_at_send(tmp_path):
    """Regression guard: the new sender must never emit AT+SEND."""
    port = transport._BytesPort()
    port.feed("TX_DONE,1,1,102700")
    port.feed("ACK_RX,2,1,-70,7.0")
    sender = transport.TSeqHexSerialSender(port=port, session_dir=tmp_path)
    sender.transmit(seq=1, payload=b"1" * 48)
    sender.close()
    sent = bytes(port.tx).decode("ascii")
    assert "AT+SEND" not in sent
    assert "AT+" not in sent


def test_t_seq_hex_sender_matches_ack_past_seq_255(tmp_path):
    """The four-byte radio sequence and serial ACK must not wrap at 255."""
    port = transport._BytesPort()
    port.feed("TX_DONE,99000,256,102700")
    port.feed("ACK_RX,99160,256,-65,9.0")
    sender = transport.TSeqHexSerialSender(port=port, session_dir=tmp_path)
    ack = sender.transmit(seq=256, payload=b"2" * 48)
    sender.close()
    assert ack is not None, "transport must match the ack when MR echoes full seq"
    assert ack.seq == 256
    assert ack.device_uptime_ms == 99160
    sent = bytes(port.tx).decode("ascii")
    assert sent.startswith("T,256,"), sent


def test_local_tx_done_without_remote_ack_is_not_delivery(tmp_path):
    port = transport._BytesPort()
    port.feed("TX_DONE,10,3,102700")
    port.feed("ACK_TIMEOUT,2010,3")
    sender = transport.TSeqHexSerialSender(port=port, session_dir=tmp_path)
    ack = sender.transmit(seq=3, payload=b"3" * 48)
    sender.close()
    assert ack is None
    assert "ACK_TIMEOUT" in (tmp_path / "timing_log.csv").read_text()


def test_ack_before_tx_done_is_rejected_as_stale(tmp_path):
    port = transport._BytesPort()
    port.feed("ACK_RX,9,4,-60,10.0")
    port.feed("TX_DONE,10,4,102700")
    port.feed("ACK_TIMEOUT,2010,4")
    sender = transport.TSeqHexSerialSender(port=port, session_dir=tmp_path)
    ack = sender.transmit(seq=4, payload=b"4" * 48)
    sender.close()
    assert ack is None
