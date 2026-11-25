# Historical hardware datasets

This document describes hardware datasets from earlier development stages of the project and their status in the current release.

## Current release

The current public reproducibility package provides the simulation datasets, analysis scripts and available hardware experimental materials. The hardware results reported in the manuscript come from session **HWS-BACS-001** (see [`hardware/README.md`](../hardware/README.md)): transmitter and receiver serial logs, scheduler decisions, host clock records, manifests and checksums.

## Historical sessions HWS-002, HWS-003 and HWS-005 (July 2026)

These sessions recorded FIFO, BACS and BACS+ scheduler logs on separate days with an earlier scheduler configuration (trust gate 0.5, 60 s half-life). They are historical hardware datasets and are **not used for any result in the current manuscript**. Historical hardware raw records, derived scheduler logs and pose-graph files from these sessions are not included in the current public release.

A provenance audit (2026-09-24) recorded the following qualifications, which apply to any use of these records:

- The timing fields (selection, transmission and reception times) of these scheduler logs were produced by the scheduler node's software timing model rather than by radio events, so they are not radio measurements of channel delay, deferral, RSSI/SNR or airtime.
- The accompanying motion-capture trajectories require additional verification before use as ground truth.
- The pose graphs derived from these sessions were reconstructed offline by replaying recorded bags; they are not the live fused map.
- Claims based on these sessions (physical map-alignment improvement, deferral-to-delay ratio) require additional experimental verification and are not made in the current manuscript.

Independent reproduction of these historical sessions is pending; the new measured experiment (`docs/HARDWARE_EXPERIMENT_V2.md`) produced session HWS-BACS-001.

## Software test fixtures

Files labelled HWS-101 to HWS-130 in earlier archives are software test fixtures, not experimental recordings.
