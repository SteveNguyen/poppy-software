import numpy
import serial
import threading
import time
import sys

class FootPressure(threading.Thread):
    def __init__(self, port, baudrate = 57600, n_sensors = 5):
        threading.Thread.__init__(self)
        self.daemon = True
        self.s = serial.Serial(port, baudrate)
        self._ns = n_sensors
        self._values = [numpy.nan] * self._ns

    @property
    def pressor_values(self):
        return self._values

    def pressor_sum(self):
        return numpy.nansum(self._values[1:])

    def run(self):
        while True:
            try:
                l = self.s.readline()
                # We force this to catch up with any potential lag.
                while self.s.inWaiting():
                    l = self.s.readline()

                l = l.replace('\r\n', '')
                try:
                    ll = map(float, l.split(','))
                    if len(ll) == self._ns:
                        self._values = ll
                except ValueError:
                   pass
            except serial.SerialException:
                self._values = [numpy.nan] * self._ns

if __name__ == '__main__':

    p = FootPressure(sys.argv[1])
    p.start()

    while True:
        print p.pressor_values
        time.sleep(0.1)
