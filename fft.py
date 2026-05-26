import pyaudio
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

CHUNK = 2048
RATE = 48000
DEVICE_NAME = "BlackHole 2ch"

p = pyaudio.PyAudio()

# Achar o índice do BlackHole
device_index = None
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if DEVICE_NAME in info['name']:
        device_index = i
        print(f"Encontrado: {info['name']} (índice {i})")
        break

if device_index is None:
    raise RuntimeError("BlackHole não encontrado. Verifique a instalação.")

stream = p.open(format=pyaudio.paFloat32,
                channels=1,
                rate=RATE,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=CHUNK)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
freqs = np.fft.rfftfreq(CHUNK, d=1/RATE)

line1, = ax1.plot([], [], color='purple', lw=1)
line2, = ax2.plot(freqs, np.zeros(len(freqs)), color='teal', lw=1)

ax1.set_xlim(0, CHUNK)
ax1.set_ylim(-1, 1)
ax1.set_title("Sinal no tempo")
ax1.set_ylabel("Amplitude")

# ax2.set_xscale('log')
ax2.set_xlim(0, 20000)
ax2.set_ylim(0, 1)
ax2.set_title("FFT em tempo real")
ax2.set_xlabel("Frequência (Hz)")
ax2.set_ylabel("Magnitude")

def update(frame):
    data = np.frombuffer(stream.read(CHUNK, exception_on_overflow=False), dtype=np.float32)
    fft = np.abs(np.fft.rfft(data)) / CHUNK

    line1.set_data(np.arange(CHUNK), data)
    line2.set_ydata(fft)
    ax2.set_ylim(0, max(fft.max(), 0.01))
    return line1, line2

ani = animation.FuncAnimation(fig, update, interval=30, blit=True)
plt.tight_layout()
plt.show()

stream.stop_stream()
stream.close()
p.terminate()