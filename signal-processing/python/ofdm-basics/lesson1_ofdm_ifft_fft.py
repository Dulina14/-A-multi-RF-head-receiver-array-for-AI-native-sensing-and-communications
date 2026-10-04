"""Lesson 1 - OFDM transmitter = IFFT, receiver = FFT.

4 QPSK symbols are turned into a time signal with an IFFT (the OFDM
transmitter) and recovered with an FFT (the OFDM receiver).
Part A of signal-processing/docs/otfs-learning-plan.md.
"""
import numpy as np
X = np.array([1+1j, 1-1j, -1+1j, -1-1j])   # 4 QPSK symbols
x = np.fft.ifft(X) * 4                       # transmitter: build the time signal
print(x)                                     # should match the table
print(np.fft.fft(x) / 4)                     # receiver: get the symbols back
