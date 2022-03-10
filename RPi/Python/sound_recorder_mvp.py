import pyaudio
import wave

import matplotlib.pyplot as plt
import numpy as np
import sys

from scipy.fftpack import fft
from scipy.io import wavfile # get the api

"""
https://people.csail.mit.edu/hubert/pyaudio/docs/#pyaudio.Stream.__init__
Interesting library
https://www.kdnuggets.com/2020/02/audio-data-analysis-deep-learning-python-part-1.html
"""

frames_per_buffer = 128

audio = pyaudio.PyAudio()
stream = audio.open(format=pyaudio.paInt16, channels=1, rate=44100, input=True, frames_per_buffer=frames_per_buffer, input_device_index=14)

frames = []

try:
    while True:
        data = stream.read(frames_per_buffer)
        frames.append(data)
except KeyboardInterrupt:
    pass

stream.stop_stream()
stream.close()
audio.terminate()

sound_file = wave.open("my_recording.wav", "wb")
sound_file.setnchannels(1)
sound_file.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
sound_file.setframerate(44100)
sound_file.writeframes(b''.join(frames))
sound_file.close()




spf = wave.open("my_recording.wav", "r")

# Extract Raw Audio from Wav File
signal = spf.readframes(-1)
signal = np.fromstring(signal, "Int16")


# If Stereo
if spf.getnchannels() == 2:
    print("Just mono files")
    sys.exit(0)


fig, ax = plt.subplots(2)
fig.suptitle('Quick Audio Plots')


ax[0].set_title("Signal Wave...")
ax[0].plot(signal)
ax[0].grid(1)
ax[0].set_ylabel('Apmlitude')
ax[0].set_xlabel('Sample #')


fs, data = wavfile.read('my_recording.wav') # load the data
a = data
b=[(ele/2**8.)*2-1 for ele in a] # this is 8-bit track, b is now normalized on [-1,1)
c = fft(b) # calculate fourier transform (complex numbers list)
d = int(len(c)/2.0)  # you only need half of the fft list (real signal symmetry)


ax[1].set_title("FFT")
ax[1].plot(abs(c[:20000]),'r')
ax[1].grid(1)
ax[1].set_xlabel('Frequency')
ax[1].set_ylabel('Power [dB]')


# set the spacing between subplots
plt.subplots_adjust(left=0.15,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.5, 
                    hspace=0.5)
plt.show()
