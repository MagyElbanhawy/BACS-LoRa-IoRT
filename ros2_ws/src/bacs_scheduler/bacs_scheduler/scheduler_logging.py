"""Strict scheduler logging schema.

Historical physical ROS 2 runs (HWS-002 / HWS-003 / HWS-005) recorded only
four timestamps per packet: ``t_gen_ns``, ``t_selected_ns``, ``t_tx_ns`` and
``t_rx_ns``. This left a transparency gap between the scheduler's decision and
the radio's actual on-air transmission, because ``t_tx_ns`` was the host's
``transmit()`` call time, not the moment the MR firmware actually keyed the
radio.

To close this gap for new sessions, the schema has been extended with three
explicit scheduler-side timestamps:

* ``t_scheduler_enqueue_ns``  - constraint enqueued by the SLAM front-end
* ``t_scheduler_dequeue_ns``  - constraint popped by the scheduler's select()
* ``t_transport_handoff_ns``   - host handed the line to the serial port

Combined with the existing ``t_tx_ns`` (now interpreted as the MR firmware's
TX ack timestamp, normalised onto the host clock via ``clock_offsets.txt``),
this yields a fully auditable timeline for every packet.
"""
SCHEDULER_FIELDS = (
    "session", "run", "seq", "policy", "robot",
    # SLAM-side generation
    "t_gen_ns",
    # Scheduler lifecycle (NEW - closes the historical transparency gap)
    "t_scheduler_enqueue_ns",
    "t_scheduler_dequeue_ns",
    "t_transport_handoff_ns",
    # Existing transport / radio timestamps
    "t_selected_ns", "t_tx_ns", "t_rx_ns",
    # Diagnostics
    "deferral_ns", "channel", "rssi_dbm", "snr_db", "payload_hex",
)
__all__ = ["SCHEDULER_FIELDS"]
