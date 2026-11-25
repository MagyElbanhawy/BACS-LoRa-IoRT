"""LoRa Transmission Airtime Calculator (Semtech SX1262 Standard).

Calculates exact time-on-air (ToA) according to Semtech formulas implemented in
SX1261/2 transceivers and European Telecommunications Standards Institute (ETSI)
EN 300 220-1 regulations.
"""

import math
from typing import Dict, Any


def lora_time_on_air(
    payload_bytes: int,
    spreading_factor: int = 7,
    bandwidth_hz: float = 125000.0,
    coding_rate: int = 1,  # 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
    preamble_symbols: int = 8,
    explicit_header: bool = True,
    low_data_rate_optimize: bool = False,
    crc_on: bool = True,
) -> Dict[str, Any]:
    """Calculates symbol duration, preamble duration, payload duration, and total time-on-air.

    Returns:
        Dict containing timing components in seconds and milliseconds.
    """
    t_sym = (2.0 ** spreading_factor) / bandwidth_hz
    t_preamble = (preamble_symbols + 4.25) * t_sym

    ih = 0 if explicit_header else 1
    crc = 1 if crc_on else 0
    de = 1 if low_data_rate_optimize else 0
    cr = coding_rate

    num = 8 * payload_bytes - 4 * spreading_factor + 28 + 16 * crc - 20 * ih
    den = 4 * (spreading_factor - 2 * de)

    bits_ceil = math.ceil(max(num, 0) / float(den))
    n_payload = 8 + max(bits_ceil * (cr + 4), 0)
    t_payload = n_payload * t_sym

    total_airtime_s = t_preamble + t_payload

    return {
        "payload_bytes": payload_bytes,
        "spreading_factor": spreading_factor,
        "bandwidth_hz": bandwidth_hz,
        "symbol_time_ms": t_sym * 1000.0,
        "preamble_time_ms": t_preamble * 1000.0,
        "payload_time_ms": t_payload * 1000.0,
        "total_airtime_ms": total_airtime_s * 1000.0,
        "total_airtime_s": total_airtime_s,
        "payload_symbols": n_payload,
    }


if __name__ == "__main__":
    res = lora_time_on_air(52, spreading_factor=7, bandwidth_hz=125000)
    print(f"LoRa ToA for 52-byte payload at SF7/BW125: {res['total_airtime_ms']:.2f} ms")
    assert 100.0 < res["total_airtime_ms"] < 105.0
