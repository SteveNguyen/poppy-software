"""
replicate.py

This allows copying one audio source to another. This
is useful if for example you have one video player that you
want to output audio to multiple screens or speakers.

NOTE: For Windows 7, in order to have a "Stereo Mix" recording device
you have to install your audio card drivers and:
Right-Click the speakers icon -> Recording Devices -> Right-Click an audio
device -> Show Disabled Devices -> Enable Stereo Mix

Here's example output and usage in my windows 7 setup:

E:\Dropbox\dev\python\audio>replicate.py 4 8
0 - Microsoft Sound Mapper - Input
1 - Desktop Microphone (Microsoft
2 - Microphone (SoundMAX Integrated
3 - Line In (SoundMAX Integrated Di
4 - Stereo Mix (SoundMAX Integrated
5 - Microsoft Sound Mapper - Output
6 - Speakers (SoundMAX Integrated D
7 - SPDIF Interface (SoundMAX Integ
8 - LG TV-1 (NVIDIA High Definition
9 - Digital Output (SoundMAX Integr
--- src ---
{'defaultHighInputLatency': 0.4,
 'defaultHighOutputLatency': 0.4,
 'defaultLowInputLatency': 0.2,
 'defaultLowOutputLatency': 0.2,
 'defaultSampleRate': 44100.0,
 'hostApi': 0,
 'index': 4,
 'maxInputChannels': 2,
 'maxOutputChannels': 0,
 'name': 'Stereo Mix (SoundMAX Integrated',
 'structVersion': 2}
--- dst ---
{'defaultHighInputLatency': 0.4,
 'defaultHighOutputLatency': 0.4,
 'defaultLowInputLatency': 0.2,
 'defaultLowOutputLatency': 0.2,
 'defaultSampleRate': 44100.0,
 'hostApi': 0,
 'index': 8,
 'maxInputChannels': 0,
 'maxOutputChannels': 2,
 'name': 'LG TV-1 (NVIDIA High Definition',
 'structVersion': 2}

Expected buffer delay: 243.8 ms (0.0, 11.6, 232.2)
Replicating audio
"""

import sys
import pprint

import pyaudio

CHUNK = 512
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100
RECORD_SECONDS = 5

p = pyaudio.PyAudio()
count = p.get_device_count()
devices = []
for i in range(count):
    devices.append(p.get_device_info_by_index(i))

for i, dev in enumerate(devices):
    print "%d - %s" % (i, dev['name'])


if len(sys.argv) < 3:
    input_device_index = int(raw_input('Choose src: '))
    output_device_index = int(raw_input('Choose dst: '))
else:
    input_device_index = int(sys.argv[1])
    output_device_index = int(sys.argv[2])
    

print '--- src ---'
pprint.pprint(devices[input_device_index])
print '--- dst ---'
pprint.pprint(devices[output_device_index])


src = p.open(format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                input = True,
                input_device_index = input_device_index,
                frames_per_buffer = CHUNK)

dst = p.open(format = FORMAT,
                channels = CHANNELS,
                rate = RATE,
                output = True,
                output_device_index = output_device_index,
                frames_per_buffer = CHUNK)


                
print ""
src_latency = 1000.0 * dst.get_input_latency()
buffer_latency = 1000.0 * CHUNK / RATE
dst_latency = 1000.0 * dst.get_output_latency()
total_latency = buffer_latency + dst_latency + src_latency
print "Expected delay: %0.1f ms (%0.1f, %0.1f, %0.1f)" % (
        total_latency, src_latency, buffer_latency, dst_latency)

print "Replicating audio"

while True:
    data = src.read(CHUNK)
    dst.write(data, CHUNK)

print "* done"

src.stop_stream()
src.close()
dst.stop_stream()
dst.close()
p.terminate()
