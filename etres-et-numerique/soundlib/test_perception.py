#Sound analysis modules
from recorder import Recorder
from analyse import spectro
from analyse import Perception

import numpy as np

import wave
import sys

import pyaudio
import time
import matplotlib.pylab as plt


class AudioFile:
    chunk = 1024

    def __init__(self, file):
        """ Init audio stream """
        self.wf = wave.open(file, 'rb')
        self.data = []
        self.rate = self.wf.getframerate()

    def play(self):
        """ Play entire file """
        data = self.wf.readframes(self.chunk)
        while data != '':
            self.data.append(data)
            data = self.wf.readframes(self.chunk)

    def close(self):
        """ Graceful shutdown """
        self.stream.close()
        self.p.terminate()

    def get_data(self):
        data = self.data
        self.data = []
        return np.fromstring(''.join(data), 'int16')




if __name__ == '__main__':

    #from wav
    # a = AudioFile(sys.argv[1])
    # a.play()
    # DATA = spectro(a.rate, a.get_data())

    #From sound device input number 0
    r = Recorder(0)
    r.recording = True
    print "record"
    time.sleep(5)
    DATA = spectro(r.rate, r.get_data())
    print "data"
    print DATA.shape


    Perceipt=Perception.load_fromfile('../sounddata/dico.npz')
    s=Perceipt.perceive(DATA)

    print "Perceived: ", s
    r.exit()

    plt.imshow(DATA)
    plt.show()
