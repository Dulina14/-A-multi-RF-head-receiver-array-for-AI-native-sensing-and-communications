"""Lesson 2 - cyclic prefix, multipath channel and one-tap equalizer.

Adds a 1-sample cyclic prefix, passes the OFDM symbol through a 2-path
channel (direct path + echo 1 sample late with gain 0.5), removes the CP,
takes the FFT and divides by the channel frequency response H[k].
The output equals the transmitted symbols.
"""
import numpy as np
M, L = 4, 1
X  = np.array([1+1j, 1-1j, -1+1j, -1-1j])
x  = np.fft.ifft(X) * M
tx = np.concatenate([x[-L:], x])                  # add CP
stream = np.concatenate([[7], tx])                # 7 = last sample of previous symbol
rx = stream[1:] + 0.5 * stream[:-1]               # direct + echo (1 sample late, gain 0.5)
Y  = np.fft.fft(rx[L:]) / M                       # remove CP, FFT
H  = 1 + 0.5 * np.exp(-2j*np.pi*np.arange(M)/M)
print(Y / H)                                      # should give X back