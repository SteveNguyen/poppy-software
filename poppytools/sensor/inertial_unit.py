import serial
import threading

import time

GYRO_X_CAL =- 26.691
GYRO_Y_CAL =- 49.95
GYRO_Z_CAL = 25.779


class Imu(threading.Thread):
    def __init__(self, port, baudrate = 57600):
        threading.Thread.__init__(self)
        self.daemon = True
        self.s = serial.Serial(port, baudrate)

        self.acc = Vector()
        self.tilt = Vector()
        self.gyro = Vector()
        self.magneto = Vector()

    def run(self):
        while True:
            l = self.s.readline()
            # We force this to catch up with any potential lag.
            # while self.s.inWaiting():
            #     l = self.s.readline()


            l = l.replace('\r\n', '')

            try:

                #just get the calibrated values and the angles
                if l[:5] == '#A-C=':
                    l = l.replace('#A-C=', '')
                    self.acc.x, self.acc.y, self.acc.z = map(float, l.split(','))

                elif l[:5] == '#G-C=':
                    l = l.replace('#G-C=', '')
                    self.gyro.x, self.gyro.y, self.gyro.z = map(float, l.split(','))

                    #test calib
                    self.gyro.x -= GYRO_X_CAL
                    self.gyro.y -= GYRO_Y_CAL
                    self.gyro.z -= GYRO_Z_CAL


                elif l[:5] == '#M-C=':
                    l = l.replace('#M-C=', '')
                    self.magneto.x, self.magneto.y, self.magneto.z = map(float, l.split(','))

                # if l[:5] == '#A-R=':
                #     l = l.replace('#A-R=', '')
                #     self.acc.x, self.acc.y, self.acc.z = map(float, l.split(','))

                # elif l[:5] == '#G-R=':
                #     l = l.replace('#G-R=', '')
                #     self.gyro.x, self.gyro.y, self.gyro.z = map(float, l.split(','))

                # elif l[:5] == '#M-R=':
                #     l = l.replace('#M-R=', '')
                #     self.magneto.x, self.magneto.y, self.magneto.z = map(float, l.split(','))



                elif l[:5] == '#YPR=':
                    l = l.replace('#YPR=', '')
                    self.tilt.x, self.tilt.y, self.tilt.z = map(float, l.split(','))

            except ValueError:
                # Some lines sent by the Arduino just seems incoherent...
                pass


class Vector(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @property
    def json(self):
        return [self.x, self.y, self.z]

    def __repr__(self):
        return '[{}, {}, {}]'.format(self.x, self.y, self.z)



if __name__ == '__main__':

    imu = Imu('/dev/poppy_imu')
    imu.start()

    while True:
        print imu.acc,  imu.gyro, imu.tilt
        time.sleep(0.1)
