# OTFS drone-sensing processor on FPGA — work plan

[← back to signal processing](../README.md)

Part of FYP Group 20: "A multi-RF head receiver array for AI-native sensing and communications".
This part (Dulina): the digital signal processing for the 5.7–6.0 GHz patch-array channel, on a Xilinx Virtex FPGA, using ISFFT-based OTFS (the OFDM-style OTFS, not Zak-OTFS).
Written 2 Oct 2026.

## 1. Words used in this plan

- **Sample**: one number the ADC produces. With I/Q (complex) sampling, each sample is a pair of numbers (I and Q).
- **Delay**: how long the echo takes to come back. Delay tells you the **range** (distance) of the drone: range = c × delay / 2.
- **Doppler**: the small frequency shift caused by movement. Doppler tells you the **velocity** of the drone: v = λ × Doppler / 2 (λ = 5.13 cm at 5.85 GHz).
- **Micro-Doppler**: extra, fast-changing Doppler caused by the spinning propellers. It is the "fingerprint" that separates drones from birds.
- **Time–frequency (TF) grid**: a table with N rows (OFDM symbols, i.e. time slots) and M columns (subcarriers, i.e. frequencies). One number per cell.
- **Delay–Doppler (DD) grid**: a table of the same size, but the rows are Doppler bins and the columns are delay bins. OTFS puts the data symbols here.
- **ISFFT** (inverse symplectic finite Fourier transform): turns the DD grid into the TF grid. In practice: one FFT/IFFT along the rows and one along the columns.
- **SFFT**: the reverse, TF grid → DD grid.
- **Heisenberg transform**: turns the TF grid into the actual time signal. For ISFFT-OTFS this is just an OFDM transmitter: IFFT per row + cyclic prefix.
- **Wigner transform**: the receiver reverse of Heisenberg = an OFDM receiver: remove cyclic prefix + FFT per row.
- **Cyclic prefix (CP)**: a copy of the end of each symbol put in front of it. For radar it also sets the maximum range that can be handled cleanly.
- **Range–Doppler map**: a 2D picture where a bright spot means "something at this distance moving at this speed".
- **CFAR** (constant false alarm rate): the rule that decides which bright spots are real targets — each cell is compared with the average noise of the cells around it.
- **BRAM**: small memory blocks inside the FPGA. The grids above live here while being processed.
- **Corner turn (transpose)**: writing a grid row by row, then reading it column by column. Needed between the two FFT directions.
- **Golden model**: a floating-point MATLAB/Python version of the whole chain. Every FPGA block is checked against it.

## 2. The processing chain and where data lives

```
Patch array → RF front end (teammates) → ADC
                                          │  samples (I/Q numbers)
  ┌───────────────────── FPGA ────────────▼───────────────────────────┐
  │ 1. DDC: shift to 0 Hz, filter, lower the sample rate              │
  │ 2. Frame sync: find where each OTFS frame starts                  │
  │ 3. Remove CP + FFT per symbol (Wigner)  → TF grid Y[n,m] in BRAM  │
  │ 4. Remove known transmitted symbols: Y·conj(X) or Y/X             │
  │      (X = the transmitted TF grid, stored in BRAM/ROM)            │
  │ 5. IFFT along subcarriers → delay (range) bins                    │
  │    FFT along symbols      → Doppler (velocity) bins               │
  │      (corner-turn memory between the two)  → range–Doppler map    │
  │ 6. CFAR detection → target list (range, velocity, power)          │
  │ 7. For the target's range bin: keep slow-time samples over many   │
  │    frames → micro-Doppler spectrogram data                        │
  └───────────────────────────────┬───────────────────────────────────┘
                                  ▼ Ethernet / USB
                     PC: display + AI classifier (drone / bird / type)
```

Transmit side (if our system also transmits): DD-domain symbols → ISFFT → IFFT + CP → DAC. For radar the frame is known in advance, so the first version can be computed in MATLAB, stored in BRAM, and played out again and again.

## 3. Example numbers (to agree with the team)

λ = 5.13 cm at 5.85 GHz. CP = 25 % of symbol length.

| Bandwidth | M (subcarriers) | N (symbols) | Range resolution | Max range set by CP | Frame length | Velocity resolution per frame |
|---|---|---|---|---|---|---|
| 100 MHz | 1024 | 64 | 1.5 m | 384 m | 0.82 ms | 31 m/s |
| 200 MHz | 1024 | 128 | 0.75 m | 192 m | 0.82 ms | 31 m/s |
| 300 MHz | 2048 | 128 | 0.5 m | 256 m | 1.09 ms | 23 m/s |

Main finding: **one frame (~1 ms) is far too short to measure drone speed** (drones fly at 0–20 m/s). About 51 ms of observation is needed for 0.5 m/s resolution. So the design must process **many frames in a row** (a "dwell" of ~50 frames or more), and the micro-Doppler spectrogram is built across frames. This affects memory size on the FPGA and must be planned from the start.

## 4. Phases

**Phase 0 — Learn the basics (2–3 weeks)**
OFDM → ISFFT-OTFS → radar basics (range, Doppler, range–Doppler map, CFAR, micro-Doppler).
Use: Hong/Thaj/Viterbo book + Monash code, MATLAB OTFS example, Raviteja 2019 radar paper, Gaudio 2020 paper.
Output: a short note in my own words + a block diagram I can explain in the viva.

**Phase 1 — Fix the system numbers (1 week)**
Choose bandwidth, M, N, CP, frames per dwell, target range. Check against the team's link-budget/SNR work.
Output: one parameter table signed off by supervisors.

**Phase 2 — Floating-point golden model (3–4 weeks)**
MATLAB or Python: OTFS TX → channel with a drone (body + spinning blades) + noise → RX chain (steps 3–7 above) → range–Doppler map → CFAR → spectrogram.
Output: plots of the range–Doppler map and micro-Doppler; saved test vectors (input samples and expected outputs).

**Phase 3 — Fixed-point model (2 weeks)**
Decide the number of bits at each stage and FFT scaling. Compare against the golden model (target: small, known error).

**Phase 4 — FPGA blocks in Vivado (6–8 weeks)**
Build and test one block at a time with testbenches fed by the Phase 2/3 vectors:
FFT (Xilinx FFT IP) → CP removal → symbol removal (complex multiply) → corner-turn memory → second FFT → CFAR (HDL or HLS) → output to PC.
Use AXI-Stream between blocks. Report resource use (LUT, DSP, BRAM) and clock speed.

**Phase 5 — Hardware tests (4+ weeks)**
(a) Digital loopback inside the FPGA. (b) DAC → coax cable → ADC with different cable lengths (known delays; the FIU Zak-OTFS paper validated this way). (c) Over the air with a corner reflector and a moving / rotating object (e.g. a fan for micro-Doppler). (d) Real drone.

**Phase 6 — Integration, AI and paper**
Connect to the team's RF board, stream spectrograms to the AI classifier, write the paper.

Mid-review target: Phases 0–3 finished and at least the FFT + range–Doppler part running in FPGA simulation.

## 5. Open questions (must be answered early)

1. Who transmits the OTFS signal? Our own transmitter next to the receiver (monostatic radar), a separate transmitter (bistatic), or someone else's signal? The proposal describes a receiver only.
2. Exact Virtex board, and which ADC (and DAC): sample rate, bits, interface (JESD204B/C, LVDS, FMC card)?
3. Bandwidth inside 5.7–6.0 GHz, and how far away the drones will be?
4. Sensing only, or also decode communication data on the FPGA?
5. Work split with teammates (OFDM/OTFS simulations already started by Anuradha and Peiris can be reused for Phase 2).

## 6. Papers

See [`../references.md`](../references.md).

No published work was found that does all three together (ISFFT-OTFS + drone detection + real-time FPGA). That gap is the project's contribution.
