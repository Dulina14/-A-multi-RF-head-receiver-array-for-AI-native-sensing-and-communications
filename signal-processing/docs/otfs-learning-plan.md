# OFDM → OTFS → drone sensing → FPGA: learning plan

[← back to signal processing](../README.md)

Started 2 Oct 2026. One lesson per step; each lesson = plain explanation + small exercise + one check question.

## Part A — OFDM (the base OTFS is built on)
1. Why OFDM exists; symbols, QAM, subcarriers, orthogonality; the transmitter is an IFFT ← lesson 1 (2 Oct 2026) – exercise: [`lesson1_ofdm_ifft_fft.py`](../python/ofdm-basics/lesson1_ofdm_ifft_fft.py)
2. Cyclic prefix, the multipath channel, the receiver FFT, one-tap equalizer – exercise: [`lesson2_cyclic_prefix_multipath.py`](../python/ofdm-basics/lesson2_cyclic_prefix_multipath.py)
3. OFDM's weakness: Doppler → inter-carrier interference (ICI)

## Part B — OTFS (ISFFT type)
4. The time–frequency grid vs the delay–Doppler grid
5. ISFFT / SFFT; OTFS = ISFFT + OFDM
6. Why the channel looks simple and steady in the delay–Doppler grid

## Part C — Sensing with OTFS
7. Radar basics: range, velocity, resolution, max range
8. Range–Doppler map from the TF grid (divide by known symbols + 2D FFT)
9. CFAR detection, micro-Doppler, drone vs bird

## Part D — Towards the FPGA
10. Fixed point, FFT IP, corner-turn (transpose) memory, AXI-Stream; mapping the chain to Virtex
