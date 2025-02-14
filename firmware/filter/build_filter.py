import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile
from scipy import signal

plt.rcParams.update({'font.sans-serif':'FreeSerif'})

f_s = 384*10**3
n_taps = 1024
f_c = 18*10**3

h = signal.firwin(n_taps, [1e3,f_c], fs=f_s, pass_zero="bandpass")
# h = signal.firwin(n_taps, 10, fs=f_s, pass_zero="highpass")
# h = signal.firwin(n_taps, f_c, fs=f_s)
H = np.fft.fft(h)

string = ""
for i in range(64):
    string = string + str(np.round(h[i]*2**15).astype(int)) + ","
    if i%8 == 7:
        string = string + "\n"
print(string)

fig, ax = plt.subplots()
ax.plot(h, "k.-")
ax.set_xlabel("$t$")
ax.set_ylabel("$h(t)$")
ax.set_title("The Filter's Impulse-Response")
plt.show()

fig, ax = plt.subplots()
ax.semilogx(np.fft.fftfreq(n_taps, 10**3/f_s), 20*np.log10(np.abs(H)), "kx-")
ax.plot([18,18], [-110,5], "k--")
ax.plot([24,24], [-110,5], "k", linestyle="dotted")
ax.fill_betweenx([-110,5],0,24,color="k",alpha=0.1)
ax.legend(["Response","approx. -3 dB Cut-off", "approx. -20 db Cut-off", "Band of Interest"])
ax.set_xlabel("Frequency (kHz)")
ax.set_ylabel("Gain $|H|$ (dB)")
ax.set_title("The Digital Filter's Frequency-Response")
plt.show()