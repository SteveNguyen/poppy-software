import numpy as np
from recorder import Recorder
from analyse import spectro

DATA = None

rec = Recorder()
rec.recording = True

DATA = spectro(rec.rate, rec.get_data())

for i in range(1000):
    DATA = np.vstack([DATA, spectro(rec.rate, rec.get_data())])

rec.recording = False
rec.exit()

np.savez('/tmp/son.npz', DATA)
