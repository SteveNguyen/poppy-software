import math
import numpy

import pypot.primitive
import transformations as tf

# import time
import pypot.utils.pypot_time as time


L_FOOT = 0.12
L_HEEL = 0.03
H_FOOT = 0.038
L_LEG = 0.1776
L_THIGH = 0.1813

L_FOOT_Z = 0.035
L_HIP_Z = 0.045
L_ABS_Z = 0.04
L_BUST_Z = 0.12


L_ARM = 0.15
L_FOREARM = 0.12

L1 = L_LEG
L2 = L_THIGH
HMAX = L1 + L2 - 0.000001

HIP_ANKLE = 0.38351
HIP_HIP = 0.04508
HIP_OFF = 3.1481


MX28_MASS = 0.072
MX64_MASS = 0.126


COMPENSATE_THRES = 0.05
# COMPENSATE_THRES=0.0025
# COMPENSATE_EPSI=0.0015
COMPENSATE_EPSI = 0.05


COMPENSATE_TIMER = 20
# COMPENSATE_TIMER=100


class SagitallHipMotion(pypot.primitive.Primitive):

    def init(self, robot):
        pypot.primitive.Primitive.__init__(self, robot)

        self._x = 0
        self._y = 0
        self._theta = 0
        self._duration = 1

    def run(self):
        mouv_hip(self.robot, self._x, self._y, self._theta, self.duration)

    def mouv_x(self, value):
        self._x += value
        self.start()

    def mouv_y(self, value):
        self._y += value
        self.start()

    def mouv_theta(self, value):
        self._theta += value
        self.start()

    def set_duration(self, value):
        self._duration = max(value, 0)


def ikin_sagitall_hip(x, y, theta=0):

    y += L_LEG + L_THIGH

    if math.sqrt(x ** 2 + y ** 2) > L_LEG + L_THIGH:
        # print " IK cannot reach the desired target "
        r = math.sqrt(x ** 2 + y ** 2)
        alpha_desired = 2 * numpy.arctan(y / (x + r))
        x = (L_LEG + L_THIGH) * numpy.sin(alpha_desired)
        y = (L_LEG + L_THIGH) * numpy.cos(alpha_desired)

    sol_ankle = numpy.real(math.degrees(-2 * numpy.arctan(
        (2 * L_LEG * x + numpy.lib.scimath.sqrt(- L_LEG ** 4 + 2 * L_LEG ** 2 * L_THIGH ** 2 + 2 * L_LEG ** 2 * x ** 2
                                                + 2 * L_LEG ** 2 * y ** 2 - L_THIGH ** 4 +
                                                2 * L_THIGH ** 2 * x ** 2
                                                + 2 * L_THIGH ** 2 * y ** 2 - x ** 4 - 2 * x ** 2 * y ** 2 - y ** 4)) / (L_LEG ** 2 + 2 * L_LEG * y - L_THIGH ** 2 + x ** 2 + y ** 2)
    )))
    sol_knee = numpy.real(math.degrees(2 * numpy.arctan(
        numpy.lib.scimath.sqrt(
            (- L_LEG ** 2 + 2 * L_LEG * L_THIGH -
             L_THIGH ** 2 + x ** 2 + y ** 2)
            * (L_LEG ** 2 + 2 * L_LEG * L_THIGH + L_THIGH ** 2 - x ** 2 - y ** 2))
        / (- L_LEG ** 2 + 2 * L_LEG * L_THIGH - L_THIGH ** 2 + x ** 2 + y ** 2))))

    sol_hip = -1 * (sol_ankle + sol_knee) + theta / 2.0
    sol_abs = theta / 2.0

    return numpy.array([sol_ankle, sol_knee, sol_hip, sol_abs])


def mouv_hip(robot, x, y, theta=0, duration=0):
    q = ikin_sagitall_hip(x, y, theta)
    duration = max(duration, 0)

    robot.goto_position({'l_ankle_y': q[0],
                         'r_ankle_y': q[0],
                         'l_knee_y': q[1],
                         'r_knee_y': q[1],
                         'l_hip_y': q[2],
                         'r_hip_y': q[2],
                         'abs_y': -q[3]},
                        duration,
                        wait=True)


def ikin_sagitall_toe(x_toe, y_toe, theta=0):
    theta = math.radians(theta)

    x = x_toe - L_FOOT * numpy.cos(theta) - H_FOOT * numpy.sin(theta)
    y = y_toe - (L_LEG + L_THIGH + H_FOOT) + H_FOOT * \
        numpy.cos(theta) - L_FOOT * numpy.sin(theta)

    if math.sqrt(x ** 2 + y ** 2) >= (L_LEG + L_THIGH):
        r = math.sqrt(x ** 2 + y ** 2)
        alpha = 2 * numpy.arctan(y / (x + r))
        x = r * numpy.cos(alpha)
        y = r * numpy.sin(alpha)

    hip_motor = numpy.real(- 2 * numpy.arctan(
        (2 * L_THIGH * x + numpy.lib.scimath.sqrt(- L_LEG ** 4 + 2 * L_LEG ** 2 * L_THIGH ** 2 + 2 * L_LEG ** 2 * x ** 2 + 2 * L_LEG **
                                                  2 * y ** 2 - L_THIGH ** 4 + 2 * L_THIGH ** 2 * x ** 2 + 2 * L_THIGH ** 2 * y ** 2 - x ** 4 - 2 * x ** 2 * y ** 2 - y ** 4))
        / (- L_LEG ** 2 + L_THIGH ** 2 - 2 * L_THIGH * y + x ** 2 + y ** 2)))
    knee_motor = numpy.real(2 * numpy.arctan(
        numpy.lib.scimath.sqrt((- L_LEG ** 2 + 2 * L_LEG * L_THIGH - L_THIGH ** 2 + x **
                                2 + y ** 2) * (L_LEG ** 2 + 2 * L_LEG * L_THIGH + L_THIGH ** 2 - x ** 2 - y ** 2))
        / (- L_LEG ** 2 + 2 * L_LEG * L_THIGH - L_THIGH ** 2 + x ** 2 + y ** 2)))
    ankle_motor = numpy.real(- knee_motor - hip_motor - theta)

    return numpy.degrees(numpy.array([hip_motor, knee_motor, ankle_motor]))


def mouv_left_toe(robot, x, y, theta=0, duration=0, wait=True):
    q = ikin_sagitall_toe(x, y, theta)
    duration = max(duration, 0)

    robot.goto_position({'l_ankle_y': q[2],
                         'l_knee_y': q[1],
                         'l_hip_y': q[0]},
                        duration,
                        wait)


def mouv_right_toe(robot, x, y, theta=0, duration=0, wait=True):
    q = ikin_sagitall_toe(x, y, theta)
    duration = max(duration, 0)

    robot.goto_position({'r_ankle_y': q[2],
                         'r_knee_y': q[1],
                         'r_hip_y': q[0]},
                        duration,
                        wait)


def sinus(ampl, t, freq=0.5, phase=0, offset=0):
    pi = numpy.pi
    return ampl * numpy.sin(freq * 2.0 * pi * t + phase * pi / 180.0) + offset


def cosinus(ampl, t, freq=0.5, phase=0, offset=0):
    pi = numpy.pi
    return ampl * numpy.cos(freq * 2.0 * pi * t + phase * pi / 180.0) + offset


# very quick and dirty
def two_links_ik_2d_cart(z, x, L1, L2):

    HMAX = L1 + L2 - 0.005

    if type(z) == float:
        h = array([z])
    z = array([HMAX if i > HMAX else i for i in z])

    cosbeta = (L1 ** 2 + L2 ** 2 - z ** 2) / (2 * L1 * L2)

    theta = arctan2(z, x)

    beta = arccos(cosbeta)

    alpha = pi / 2.0 - beta / 2.0 + \
        arctan((L2 - L1) / (L2 + L1) * cotan(beta / 2.0)) + theta

    return (alpha, beta)


def two_links_ik_2d(h, theta, L1, L2):

    HMAX = L1 + L2 - 0.005

    if type(h) == float:
        h = array([h])
    h = array([HMAX if i > HMAX else i for i in h])

    cosbeta = (L1 ** 2 + L2 ** 2 - h ** 2) / (2 * L1 * L2)

    beta = arccos(cosbeta)

    alpha = pi / 2.0 - beta / 2.0 + \
        arctan((L2 - L1) / (L2 + L1) * cotan(beta / 2.0)) + theta

    return (alpha, beta)


def two_links_fk_3d(self, alpha, beta, phi, L1, L2):

    h = sqrt(L1 ** 2 + L2 ** 2 - 2.0 * L1 * L2 * cos(beta))

    theta = -(pi / 2.0 - beta / 2.0 +
              arctan((L2 - L1) / (L2 + L1) * cotan(beta / 2.0))) + alpha

    x = h * sin(phi)

    y = h * sin(theta)

    return (x, y)


def leg_up(l, theta=0.0):
    # fixme for 0
    if l == 0:
        alpha = theta
        beta = pi
    else:
        alpha, beta = leg_ik_2d(HMAX - l, theta)
    # return (alpha, beta - pi, (pi-beta)/2.0) #FIXME ANKLE
    return (alpha, beta - pi, (pi - beta) - alpha)


# compute the quadrilateral
def get_gproj(theta2, db):

    # a=0.40414
    # b=0.0396

    # new version

    a = HIP_ANKLE
    b = HIP_HIP

    s = sqrt(db ** 2 + a ** 2 - 2 * a * db * cos(theta2))

    # psi=arccos((b**2+s**2-a**2)/(2*b*s))
    # beta=arccos((db**2+s**2-a**2)/(2*db*s))
    # lambd=arccos((a**2+s**2-b**2)/(2*a*s))

    cospsi = ((b ** 2 + s ** 2 - a ** 2) / (2 * b * s))
    cosbeta = ((db ** 2 + s ** 2 - a ** 2) / (2 * db * s))
    coslambd = ((a ** 2 + s ** 2 - b ** 2) / (2 * a * s))

    # print s, cospsi, coslambd

    sinpsi = sqrt(1 - cospsi ** 2)
    sinbeta = sqrt(1 - cosbeta ** 2)
    sinlambd = sqrt(1 - coslambd ** 2)

    psi = arctan2(sinpsi, cospsi)
    beta = arctan2(sinbeta, cosbeta)
    lambd = arctan2(sinlambd, coslambd)

    theta3 = psi - beta
    theta4 = pi - lambd - beta

    thetaA = psi + (pi - theta2 - beta)
    thetaB = 2 * pi - thetaA - theta2 - (pi - theta4)

    Cx = a * cos(theta2) + b / 2 * (cos(psi - beta))

    return Cx, theta3, theta4, thetaA, thetaB


def get_inv_gproj(cx, db):

    # if cx==0.075:
    #     cx=(self.r_foot_ref_com[0]-self.l_foot_ref_com[0])/2.0

    # just a security
    # if cx < 0.025:
    #     cx = 0.025
    # elif cx > 0.075 + 0.025:
    #     cx = 0.075 + 0.025

    theta2 = arange(0.0, pi, 0.001)

    # db=(self.r_foot_ref_com[0]-self.l_foot_ref_com[0])
    # print db

    # db = 0.15

    # cx = db / 2.0

    C, theta3, theta4, thetaA, thetaB = get_gproj(theta2, db)

    # bullshit, TODO replace that with a Newton method
    it = 0
    mini = 99999999
    Cx_idx = 0
    for i in C:
        if not isnan(i):
            if abs(i - cx) < mini:
                mini = abs(i - cx)
                Cx_idx = it
        it += 1

    # print C[Cx_idx]

    return C[Cx_idx], theta2[Cx_idx], pi - theta4[Cx_idx], thetaA[Cx_idx], thetaB[Cx_idx], theta3[Cx_idx]


def get_inv_gproj_angles(robot, cgoal):

    db, =  get_db(robot)
    c, t1, t2, t3, t4, t5 = get_inv_gproj(cgoal + db / 2.0, db)

    return {'l_ankle_x': degrees(t2 - pi / 2.0) - HIP_OFF,  # 4deg?
            'r_ankle_x': degrees(t1 - pi / 2.0) + HIP_OFF,
            'l_hip_x': degrees(t4 - pi / 2.0) - HIP_OFF,
            'r_hip_x': degrees(pi / 2.0 - t3) + HIP_OFF,
            'abs_x': degrees(- t5)}


class MoveGprojSin(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, amp, freq,  arms=False, db=0.15, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.amp = amp
        self.freq = freq
        self.db = db
        self.arms = arms
        self.val = 0
        self.t = 0
        self.prevt = 0
        self.dt = 0

        self.prev_sig = 0
        self.left = False
        self.right = False

    def SetAmp(self, amp):
        self.amp = amp

    def SetFreq(self, freq):
        self.freq = freq

    def GetAmp(self):
        return self.amp

    def GetFreq(self):
        return self.freq

    def update(self):

        self.dt = self.elapsed_time - self.prevt
        self.prevt = self.elapsed_time

        self.t += self.dt
        # print self.t
        self.val = sinus(self.amp, self.t, self.freq)

        c, t1, t2, t3, t4, t5 = get_inv_gproj(
            self.val + self.db / 2.0, self.db)
        # print  sinus(self.amp, t, self.freq)

        # print self.val
        # FIXME
        # self.robot.abs_z.goal_position =- self.val * 500

        # self.robot.l_ankle_x.goal_position = degrees(t2 - pi / 2.0) - HIP_OFF
        # self.robot.r_ankle_x.goal_position = degrees(t1 - pi / 2.0) + HIP_OFF

        self.robot.l_hip_x.goal_position = degrees(t4 - pi / 2.0) - HIP_OFF
        self.robot.r_hip_x.goal_position = degrees(pi / 2.0 - t3) + HIP_OFF
        self.robot.abs_x.goal_position = degrees(- t5)

        # test arms
        if self.arms:

            marms = cosinus(30, t, self.freq)

            if marms < 0:
                self.robot.r_shoulder_x.goal_position = marms - 10
            else:
                self.robot.l_shoulder_x.goal_position = marms + 10

        sig = sign(cosinus(1.0, self.t, self.freq))

        if sig != self.prev_sig:  # cross 0

            # go to negative=go to the right
            if self.prev_sig > 0 and self.right == False:
                self.right = True
                self.left = False

            elif self.prev_sig < 0 and self.left == False:
                self.right = False
                self.left = True

            else:
                self.right = False
                self.left = False

        self.prev_sig = sig


class MoveGprojSinHip(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, amp, freq,  arms=False, db=0.15, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.amp = amp
        self.freq = freq
        self.db = db
        self.arms = arms
        self.val = 0
        self.t = 0
        self.prevt = 0
        self.dt = 0

        self.hip_amp = 1.5

        self.prev_sig = 0
        self.left = False
        self.right = False

        self.move_forward = False
        self.turn = False

    def SetAmp(self, amp):
        self.amp = amp

    def SetFreq(self, freq):
        self.freq = freq

    def GetAmp(self):
        return self.amp

    def GetFreq(self):
        return self.freq

    def update(self):

        self.dt = self.elapsed_time - self.prevt
        self.prevt = self.elapsed_time

        self.t += self.dt
        # print self.t
        self.val = sinus(self.amp, self.t, self.freq)
        # turnangle =  sinus(10.0, self.t, self.freq)

        v = sinus(self.hip_amp, self.t, self.freq)

        if self.move_forward == True:

            self.robot.l_hip_y.goal_position = - v
            self.robot.r_hip_y.goal_position = v
        else:
            self.robot.l_hip_y.goal_position = 0.0
            self.robot.r_hip_y.goal_position = 0.0
            # self.robot.goto_position({'l_hip_y': 0, 'r_hip_y': 0}, 0.3, wait = False)

        if self.turn == 'right':
            turnangle = sinus(10.0, self.t, self.freq)

            self.robot.r_hip_z.goal_position = turnangle - 10
            # self.robot.l_hip_z.goal_position =- turnangle + 10

        elif self.turn == 'left':
            turnangle = - sinus(10.0, self.t, self.freq)
            self.robot.l_hip_z.goal_position = - turnangle + 10

        else:
            # self.robot.goto_position({'l_hip_z': 0, 'r_hip_z': 0}, 0.3, wait = False)
            self.robot.l_hip_z.goal_position = 0
            self.robot.r_hip_z.goal_position = 0

        print self.move_forward, self.turn

        c, t1, t2, t3, t4, t5 = get_inv_gproj(
            self.val + self.db / 2.0, self.db)
        # print  sinus(self.amp, t, self.freq)

        # print self.val
        # FIXME
        # self.robot.abs_z.goal_position =- self.val * 500

        self.robot.l_ankle_x.goal_position = degrees(t2 - pi / 2.0) - HIP_OFF
        self.robot.r_ankle_x.goal_position = degrees(t1 - pi / 2.0) + HIP_OFF
        self.robot.l_hip_x.goal_position = degrees(t4 - pi / 2.0) - HIP_OFF
        self.robot.r_hip_x.goal_position = degrees(pi / 2.0 - t3) + HIP_OFF
        self.robot.abs_x.goal_position = degrees(- t5)

        # test arms
        # if self.arms:

        #     marms = cosinus(30, self.t, self.freq)

        #     if marms < 0:
        #         self.robot.r_shoulder_x.goal_position = marms - 10
        #     else:
        #         self.robot.l_shoulder_x.goal_position = marms + 10

        if self.arms:

            marms = sinus(10, self.t, self.freq)

            self.robot.r_shoulder_y.goal_position = - marms

            self.robot.l_shoulder_y.goal_position = marms

        sig = sign(cosinus(1.0, self.t, self.freq))

        if sig != self.prev_sig:  # cross 0

            # go to negative=go to the right
            if self.prev_sig > 0 and self.right == False:
                self.right = True
                self.left = False

            elif self.prev_sig < 0 and self.left == False:
                self.right = False
                self.left = True

            else:
                self.right = False
                self.left = False

        self.prev_sig = sig


class MoveGprojOSC(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, amp, freq,  arms=False, db=0.15, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.amp = amp
        self.freq = freq
        self.db = db
        self.arms = arms
        self.val = 0
        self.t = 0
        self.prevt = 0
        self.dt = 0

        self.prev_sig = 0
        self.left = False
        self.right = False

    def SetAmp(self, amp):
        self.amp = amp

    def SetFreq(self, freq):
        self.freq = freq

    def GetAmp(self):
        return self.amp

    def GetFreq(self):
        return self.freq

    def update(self):

        self.dt = self.elapsed_time - self.prevt
        self.prevt = self.elapsed_time

        self.t += self.dt
        # print self.t
        self.val = sinus(self.amp, self.t, self.freq)

        sig = sign(cosinus(1.0, self.t, self.freq))

        if sig != self.prev_sig:  # cross 0

            # go to negative=go to the right
            if self.prev_sig > 0 and self.right == False:
                self.right = True
                self.left = False

            elif self.prev_sig < 0 and self.left == False:
                self.right = False
                self.left = True

            else:
                self.right = False
                self.left = False

        self.prev_sig = sig

        c, t1, t2, t3, t4, t5 = get_inv_gproj(
            sig * self.amp + self.db / 2.0, self.db)
        # print  sinus(self.amp, t, self.freq)

        # print self.val
        # FIXME
        # self.robot.abs_z.goal_position =- self.val * 500

        # self.robot.l_ankle_x.goal_position = degrees(t2 - pi / 2.0) - HIP_OFF
        # self.robot.r_ankle_x.goal_position = degrees(t1 - pi / 2.0) + HIP_OFF
        # self.robot.l_hip_x.goal_position = degrees(t4 - pi / 2.0) - HIP_OFF
        # self.robot.r_hip_x.goal_position = degrees(pi / 2.0 - t3) + HIP_OFF
        # self.robot.abs_x.goal_position = degrees( - t5)

        self.robot.goto_position({'l_ankle_x': degrees(t2 - pi / 2.0) - HIP_OFF,
                                  'r_ankle_x': degrees(t1 - pi / 2.0) + HIP_OFF,
                                  'l_hip_x': degrees(t4 - pi / 2.0) - HIP_OFF,
                                  'r_hip_x': degrees(pi / 2.0 - t3) + HIP_OFF,
                                  'abs_x': degrees(- t5)},
                                 0.1,
                                 wait=False
                                 )

        # test arms
        if self.arms:

            marms = cosinus(30, t, self.freq)

            if marms < 0:
                self.robot.r_shoulder_x.goal_position = marms - 10
            else:
                self.robot.l_shoulder_x.goal_position = marms + 10


class MoveGprojSinWFeet(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, amp, freq,  arms=False, db=0.15, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.amp = amp
        self.freq = freq
        self.db = db
        self.arms = arms
        self.val = 0
        self.t = 0
        self.prevt = 0
        self.dt = 0

    def SetAmp(self, amp):
        self.amp = amp

    def SetFreq(self, freq):
        self.freq = freq

    def GetAmp(self):
        return self.amp

    def GetFreq(self):
        return self.freq

    def update(self):

        self.dt = self.elapsed_time - self.prevt
        self.prevt = self.elapsed_time

        self.t += self.dt
        # print self.t
        self.val = sinus(self.amp, self.t, self.freq)
        c, t1, t2, t3, t4, t5 = get_inv_gproj(
            self.val + self.db / 2.0, self.db)
        # print  sinus(self.amp, t, self.freq)

        # print self.val
        # FIXME
        # self.robot.abs_z.goal_position =- self.val * 500

        self.robot.l_ankle_x.goal_position = degrees(t2 - pi / 2.0) - HIP_OFF
        self.robot.r_ankle_x.goal_position = degrees(t1 - pi / 2.0) + HIP_OFF
        self.robot.l_hip_x.goal_position = degrees(t4 - pi / 2.0) - HIP_OFF
        self.robot.r_hip_x.goal_position = degrees(pi / 2.0 - t3) + HIP_OFF
        self.robot.abs_x.goal_position = degrees(- t5)

        # test arms
        if self.arms:

            marms = cosinus(30, t, self.freq)

            if marms < 0:
                self.robot.r_shoulder_x.goal_position = marms - 10
            else:
                self.robot.l_shoulder_x.goal_position = marms + 10


class StabilizeTrunkY(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, imu, offset=0, kp=1.0 / 20.0, kd=1.0 / 2000.0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.imu = imu
        self.pos = 0
        self.Kp = kp
        self.Kd = kd
        # self.Kd = 0

        self.offset = offset

        self.prev_y = 0
        self.prev_t = 0

        self.limit = 45.

    def update(self):

        tilt_y = (self.imu.tilt.z - self.offset)
        # gyro_y = 0.0
        # if abs(gyro_y) > 30.0:
        #     gyro_y = self.imu.gyro.z

        # dtilt_y = (tilt_y - self.prev_y) / (self.elapsed_time - self.prev_t)
        # self.prev_t = self.elapsed_time
        # self.prev_y = tilt_y

        dtilt_y = self.imu.gyro.x

        self.pos -= tilt_y * self.Kp + dtilt_y * self.Kd
        # print self.imu.tilt.z, tilt_y, self.pos, self.imu.gyro#, dtilt_y
        # print self.imu.gyro
        if abs(self.pos) < self.limit:
            self.robot.abs_y.goal_position = self.pos
        else:
            self.robot.abs_y.goal_position = sign(self.pos) * self.limit


class StabilizeTrunkX(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, imu, offset=0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.imu = imu
        self.pos = 0
        self.Kp = 1.0 / 30.0
        self.Kd = 1.0 / 2000.0
        # self.Kd = 0

        self.offset = offset

        self.prev_x = 0
        self.prev_t = 0

    def update(self):

        tilt_x = (self.imu.tilt.y - self.offset)
        # gyro_y = 0.0
        # if abs(gyro_y) > 30.0:
        #     gyro_y = self.imu.gyro.z

        # dtilt_y = (tilt_y - self.prev_y) / (self.elapsed_time - self.prev_t)
        # self.prev_t = self.elapsed_time
        # self.prev_y = tilt_y

        dtilt_x = self.imu.gyro.y

        self.pos -= tilt_x * self.Kp + dtilt_x * self.Kd
        print self.imu.tilt.y, tilt_x, self.pos, self.imu.gyro  # , dtilt_y

        # print self.imu.tilt, self.imu.gyro#, dtilt_y
        # print self.imu.gyro
        if abs(self.pos) < 25:
            self.robot.abs_x.goal_position = self.pos


class StabilizeTrunkYHip(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, imu, offset=0, kp=1.0 / 20.0, kd=1.0 / 2000.0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.imu = imu
        self.pos = 0
        self.Kp = kp
        self.Kd = kd
        # self.Kd = 0

        self.offset = offset

        self.prev_y = 0
        self.prev_t = 0

        self.state = 'both'
        self.limit = 50

    def update(self):

        tilt_y = (self.imu.tilt.z - self.offset)
        # gyro_y = 0.0
        # if abs(gyro_y) > 30.0:
        #     gyro_y = self.imu.gyro.z

        # dtilt_y = (tilt_y - self.prev_y) / (self.elapsed_time - self.prev_t)
        # self.prev_t = self.elapsed_time
        # self.prev_y = tilt_y

        dtilt_y = self.imu.gyro.x

        self.pos -= tilt_y * self.Kp + dtilt_y * self.Kd
        # print self.imu.tilt.z, tilt_y, self.pos, self.imu.gyro#, dtilt_y
        # print self.imu.gyro
        if self.pos > self.limit:
            self.pos = self.limit
        elif self.pos < - self.limit / 2.0:
            self.pos = - self.limit / 2.0

        # print self.pos, tilt_y, self.robot.l_hip_y.present_position,
        # self.robot.r_hip_y.present_position

        if self.state == 'both':
            self.robot.l_hip_y.goal_position = - self.pos
            self.robot.r_hip_y.goal_position = - self.pos
        elif self.state == 'left':
            self.robot.l_hip_y.goal_position = - self.pos
        elif self.state == 'right':
            self.robot.r_hip_y.goal_position = - self.pos


class StabilizeTrunkYHipAnkle(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, imu, offset=0, kp=1.0 / 20.0, kd=1.0 / 2000.0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.imu = imu
        self.pos = 0
        self.Kp = kp
        self.Kd = kd
        # self.Kd = 0

        self.offset = offset

        self.prev_y = 0
        self.prev_t = 0

        self.state = 'both'
        self.limit = 50

    def update(self):

        tilt_y = (self.imu.tilt.z - self.offset)
        # gyro_y = 0.0
        # if abs(gyro_y) > 30.0:
        #     gyro_y = self.imu.gyro.z

        # dtilt_y = (tilt_y - self.prev_y) / (self.elapsed_time - self.prev_t)
        # self.prev_t = self.elapsed_time
        # self.prev_y = tilt_y

        dtilt_y = self.imu.gyro.x

        self.pos -= tilt_y * self.Kp + dtilt_y * self.Kd
        # print self.imu.tilt.z, tilt_y, self.pos, self.imu.gyro#, dtilt_y
        # print self.imu.gyro
        if self.pos > self.limit:
            self.pos = self.limit
        elif self.pos < - self.limit / 2.0:
            self.pos = - self.limit / 2.0

        # print self.pos, tilt_y, self.robot.l_hip_y.present_position,
        # self.robot.r_hip_y.present_position

        if self.state == 'both':
            self.robot.l_hip_y.goal_position = - self.pos
            self.robot.r_hip_y.goal_position = - self.pos

            self.robot.l_ankle_y.goal_position = - self.pos / 2.0
            self.robot.r_ankle_y.goal_position = - self.pos / 2.0

        elif self.state == 'left':
            self.robot.l_hip_y.goal_position = - self.pos
            # self.robot.l_ankle_y.goal_position =- self.pos/ 2.0
            # self.robot.l_ankle_y.goal_position =- self.pos
        elif self.state == 'right':
            self.robot.r_hip_y.goal_position = - self.pos
            # self.robot.r_ankle_y.goal_position =- self.pos/ 2.0
            # self.robot.r_ankle_y.goal_position =- self.pos


class StabilizeTrunkXHip(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, imu, offset=0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.imu = imu
        self.pos = 0
        self.Kp = 1.0 / 20.0
        self.Kd = 1.0 / 2000.0
        # self.Kd = 0

        self.offset = offset

        self.prev_x = 0
        self.prev_t = 0
        self.state = 'none'

    def update(self):

        tilt_x = (self.imu.tilt.y - self.offset)
        # gyro_y = 0.0
        # if abs(gyro_y) > 30.0:
        #     gyro_y = self.imu.gyro.z

        # dtilt_y = (tilt_y - self.prev_y) / (self.elapsed_time - self.prev_t)
        # self.prev_t = self.elapsed_time
        # self.prev_y = tilt_y

        dtilt_x = self.imu.gyro.y

        self.pos -= tilt_x * self.Kp + dtilt_x * self.Kd
        # print self.imu.tilt.y, tilt_x, self.pos, self.imu.gyro#, dtilt_y

        # print self.imu.tilt, self.imu.gyro#, dtilt_y
        # print self.imu.gyro
        if self.pos < - 15:
            self.pos = - 15
        elif self.pos > 15:
            self.pos = 15

        if self.state == 'both':  # Dangerous! not so simple
            self.robot.l_hip_x.goal_position = - self.pos
            self.robot.r_hip_x.goal_position = - self.pos
        elif self.state == 'left':
            if self.pos > 2.5:
                self.pos = 2.5
            self.robot.l_hip_x.goal_position = - self.pos
        elif self.state == 'right':
            if self.pos < - 2.5:
                self.pos = - 2.5
            self.robot.r_hip_x.goal_position = - self.pos


# torque compensation (find the equilibrium point)
class Compensate(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq, motor_list,  continuous=False):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.motor_dict = motor_list

        # self.motor_list = [self.get_mockup_motor(m) for m in motor_list]

        self.timer = {}
        self.err = {}
        self.done = {}
        self.command = {}
        for k, i in self.motor_dict.iteritems():

            self.timer[k] = COMPENSATE_TIMER
            self.err[k] = 0
            self.done[k] = False
            self.command[k] = i

        self.thres = COMPENSATE_THRES
        self.epsi = COMPENSATE_EPSI
        self.continuous = continuous

    def update(self):

        for motor, goal in self.motor_dict.iteritems():
            if self.timer[motor] == 0:  # and not self.done[motor]:
                m = getattr(self.robot, motor)
                self.err[motor] = m.present_position - self.command[motor]

                if abs(self.err[motor]) > self.thres and not self.done[motor]:
                    # FIXME
                    # if motor == 'r_ankle_y':
                    #     self.command[motor] += -self.epsi * sign(self.err[motor])
                    # else:
                    #     self.command[motor] += -self.epsi * sign(self.err[motor])

                    self.command[motor] += -self.epsi * sign(self.err[motor])

                    m.goal_position = self.command[motor]

                    print motor, self.err[motor], self.command[motor]

                else:
                    if not self.continuous:
                        self.done[motor] = True

                    print motor, ' done', self.command[motor], self.err[motor]
                    if all(self.done.values()) == True:
                        # print 'all done'
                        self.stop()

                self.timer[motor] = COMPENSATE_TIMER

            else:
                self.timer[motor] -= 1


class CompensateFoot(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq, goal=3.0,  continuous=False):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        # self.motor_list = [self.get_mockup_motor(m) for m in motor_list]

        self.timer = COMPENSATE_TIMER
        self.err = 0
        self.done = False
        self.l_command = goal
        self.r_command = goal

        self.thres = COMPENSATE_THRES
        self.epsi = COMPENSATE_EPSI
        self.continuous = continuous

    def update(self):

        if self.timer == 0:  # and not self.done[motor]:

            errl = self.robot.l_ankle_y.present_position - self.l_command
            errr = self.robot.r_ankle_y.present_position - self.r_command

            self.err = (errl + errr) / 2.0

            if abs(self.err) > self.thres and not self.done:

                self.l_command += -self.epsi * sign(self.err)
                self.r_command += -self.epsi * sign(self.err)

                self.robot.l_ankle_y.goal_position = self.l_command
                self.robot.r_ankle_y.goal_position = self.r_command

                print 'foot', self.err, self.l_command

            else:
                if not self.continuous:

                    self.done = True
                    self.stop()
                print 'Foot torque compensation done:', self.l_command, self.err

                # self.stop()

            self.timer = COMPENSATE_TIMER

        else:
            self.timer -= 1


def move_gproj(robot, cx, duration=2.0, wait=True):

    db, = get_db(robot)
    c, t1, t2, t3, t4, t5 = get_inv_gproj(0.075 + cx, db)
    robot.goto_position({'l_ankle_x': degrees(t2 - pi / 2.0),
                         'r_ankle_x': degrees(t1 - pi / 2.0),
                         'l_hip_x': degrees(t4 - pi / 2.0) - HIP_OFF,
                         'r_hip_x': degrees(pi / 2.0 - t3) + HIP_OFF,
                         'abs_x': degrees(- t5)
                         },
                        duration,
                        wait=wait)


def get_db(robot):

    # distance between feet axes

    # a=0.40414
    # b=0.0396

    a = HIP_ANKLE
    b = HIP_HIP

    thetaA = radians(robot.r_hip_x.present_position - HIP_OFF) - pi / 2.0
    thetaB = radians(robot.l_hip_x.present_position + HIP_OFF) + pi / 2.0

    x = a - b * cos(thetaA) - a * cos(thetaA + pi - thetaB)
    y = - b * sin(- thetaA) + a * sin(thetaA + pi - thetaB)

    db = sqrt(x ** 2 + y ** 2)

    theta2 = math.atan2(y, x)

    return db, degrees(theta2)


class FWGeomtry():

    def __init__(self):

        # sagital
        self.RAnkleRotY = None
        self.LAnkleRotY = None

        self.RKneeRotY = None
        self.LKneeRotY = None

        self.RHipRotY = None
        self.LHipRotY = None

        self.AbsRotY = None
        self.BustRotY = None

        self.FootT = tf.translation_matrix((0, 0, L_FOOT_Z))
        self.ThighT = tf.translation_matrix((0, 0, L_THIGH))
        self.LegT = tf.translation_matrix((0, 0, L_LEG))
        self.HipT = tf.translation_matrix((0, 0, L_HIP_Z))
        self.AbsT = tf.translation_matrix((0, 0, L_ABS_Z))
        self.BustT = tf.translation_matrix((0, 0, L_BUST_Z))

        # Totally guessed com of segments

        self.SegCom = array([[0.025, 0, L_FOOT_Z, 1],
                             [0, 0, L_THIGH, 1],
                             [0, 0, L_LEG, 1],
                             [0, 0, 3.0 * L_HIP_Z / 4.0, 1],
                             [- 0.02, 0, 3.0 * L_ABS_Z / 4.0, 1],
                             [- 0.02, 0, 3.0 * L_BUST_Z / 4.0, 1]])

        # Totally guessed masses of segments (roughtly the mass of the motors)
        self.Masses = array([2 * (2 * MX28_MASS), 2 * (MX28_MASS), 2 *
                             (MX28_MASS), 2 * (2 * MX28_MASS), (2 * MX64_MASS), 13 * MX28_MASS])
        self.TotalMass = self.Masses.sum()
        self.Masses /= self.TotalMass

        self.roty = None

        self.xaxis = (1, 0, 0)
        self.yaxis = (0, 1, 0)
        self.zaxis = (0, 0, 1)

    def update_sagital_rot(self, robot):

        # ok we assume a symetric configuration with both foot on the ground

        self.LAnkleRotY = tf.rotation_matrix(
            radians(robot.l_ankle_y.present_position), self.yaxis)
        self.RAnkleRotY = tf.rotation_matrix(
            radians(robot.r_ankle_y.present_position), self.yaxis)

        self.LKneeRotY = tf.rotation_matrix(
            radians(- robot.l_knee_y.present_position), self.yaxis)
        self.RKneeRotY = tf.rotation_matrix(
            radians(- robot.r_knee_y.present_position), self.yaxis)

        self.LHipRotY = tf.rotation_matrix(
            radians(- robot.l_hip_y.present_position), self.yaxis)
        self.RHipRotY = tf.rotation_matrix(
            radians(- robot.r_hip_y.present_position), self.yaxis)

        self.AbsRotY = tf.rotation_matrix(
            radians(robot.abs_y.present_position), self.yaxis)
        self.BustRotY = tf.rotation_matrix(
            radians(robot.bust_y.present_position), self.yaxis)

        self.roty = array([tf.rotation_matrix(0, self.yaxis), (self.LAnkleRotY + self.RAnkleRotY) / 2.0,
                           (self.LKneeRotY + self.RKneeRotY) / 2.0, (self.LHipRotY + self.RHipRotY) / 2.0, self.AbsRotY, self.BustRotY])

        # self.transformy =array([tf.concatenate_matrices(self.roty[i], self.SegCom[i]) for i in range(len(self.roty))])
        # self.transformy =array([self.roty[i].dot( self.SegCom[i]) for i in range(len(self.roty))])

        self.transformy = array([identity(4),
                                 self.FootT,
                                 tf.concatenate_matrices(
            (self.LAnkleRotY + self.RAnkleRotY) / 2.0, self.ThighT),
            tf.concatenate_matrices(
            (self.LKneeRotY + self.RKneeRotY) / 2.0, self.LegT),
            tf.concatenate_matrices(
            (self.LHipRotY + self.RHipRotY) / 2.0, self.HipT),
            tf.concatenate_matrices(
            self.AbsRotY, self.AbsT),
            tf.concatenate_matrices(
            self.BustRotY, self.BustT)
        ])

        a = dot(self.transformy[1], self.transformy[2])
        b = dot(a, self.transformy[3])
        c = dot(b, self.transformy[4])
        d = dot(c, self.transformy[5])
        e = dot(d, self.transformy[6])

        self.comtrans = array([self.transformy[1],
                               a,
                               b,
                               c,
                               d,
                               e
                               ])

    def update(self, robot):
        self.update_sagital_rot(robot)

    def get_com_sagital(self):  # FIXME!!

        compos = array([dot(self.comtrans[i], self.SegCom[i])
                        * self.Masses[i] for i in range(len(self.Masses))])

        return compos.sum(axis=0)[: 3]
        # print self.transformy
        # print self.roty
        # print


class TrackComSimple(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.init()
        self.com_sagital = []

    def init(self):
        # here build the transformation matrices

        self.Geom = FWGeomtry()

    def update(self):

        self.Geom.update(self.robot)
        self.com_sagital = self.Geom.get_com_sagital()

        # print self.robot.l_ankle_y.present_position,- self.robot.l_knee_y.present_position,- self.robot.l_hip_y.present_position, self.robot.abs_y.present_position, self.robot.bust_y.present_position
        # print self.robot.r_ankle_y.present_position,- self.robot.r_knee_y.present_position,- self.robot.r_hip_y.present_position, self.robot.abs_y.present_position, self.robot.bust_y.present_position
        # print


class CtrlComSimple(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, goal=0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.com_sagital = []
        self.goalpos = goal
        self.Kp = 30.0
        self.MAX = 30.0
        self.pos = self.robot.abs_y.present_position
        self.init()

    def init(self):
        self.Geom = FWGeomtry()

    def update(self):

        self.Geom.update(self.robot)
        self.com_sagital = self.Geom.get_com_sagital()

        err = (self.com_sagital[0] - self.goalpos)

        self.pos -= self.Kp * err
        if self.pos > self.MAX:
            self.pos = self.MAX
        elif self.pos < - self.MAX:
            self.pos = - self.MAX

        print self.com_sagital[0], err, self.pos

        self.robot.abs_y.goal_position = self.pos


class SlowPID(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq, motor_list,  continuous=True, kp=None):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.motor_dict = motor_list

        # self.motor_list = [self.get_mockup_motor(m) for m in motor_list]
        if kp != None:
            self.KP = 0.05
        else:
            self.KP = kp

        self.err = {}
        # self.done = {}
        self.timer = {}
        self.command = {}
        for k, i in self.motor_dict.iteritems():

            self.timer[k] = 10
            self.err[k] = 0
            # self.done[k] = False
            self.command[k] = i

        # self.thres = COMPENSATE_THRES
        # self.epsi = COMPENSATE_EPSI
        self.continuous = continuous


class FastPID(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq, motor_list,  continuous=True, kp=None):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.motor_dict = motor_list

        # self.motor_list = [self.get_mockup_motor(m) for m in motor_list]
        if kp != None:
            self.KP = 0.05
        else:
            self.KP = kp

        self.err = {}
        # self.done = {}
        # self.timer = {}
        self.command = {}
        for k, i in self.motor_dict.iteritems():

            # self.timer[k] =10
            self.err[k] = 0
            # self.done[k] = False
            self.command[k] = i

        # self.thres = COMPENSATE_THRES
        # self.epsi = COMPENSATE_EPSI
        self.continuous = continuous

    def update(self):

        for motor, goal in self.motor_dict.iteritems():
            # if self.timer[motor] == 0:# and not self.done[motor]:
            m = getattr(self.robot, motor)
            self.err[motor] = m.present_position - goal

            self.command[motor] += - self.KP * self.err[motor]
            m.goal_position = self.command[motor]

            # print motor, self.err[motor], self.command[motor]

            # else:
            #     if not self.continuous:
            #         self.done[motor] = True

            #     print motor, ' done', self.command[motor], self.err[motor]
            #     if all(self.done.values()) == True:
            # print 'all done'
            # self.stop()

            #     self.timer[motor] = 5

            # else:
            #     self.timer[motor] -= 1

    def new_goal(self, motor, goal):
        self.motor_dict[motor] = goal


class ControlFeetDist(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, dist=0.15, strict=False, refresh_freq=50,  continuous=True, kp=100.0):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.dist = dist
        self.KP = kp

        self.err = 0
        self.lcommand = 0
        self.rcommand = 0
        self.db, a = get_db(robot)
        self.continuous = continuous
        self.strict = strict

    def update(self):

        self.db, a = get_db(self.robot)

        self.err = self.db - self.dist

        c = self.strict or (not self.strict and (self.err < 0))

        if c:
            l = self.robot.l_hip_x.present_position - self.KP * self.err
            r = self.robot.r_hip_x.present_position + self.KP * self.err

            # self.robot.l_hip_x.goal_position = self.robot.l_hip_x.present_position - self.KP * self.err
            # self.robot.r_hip_x.goal_position = self.robot.r_hip_x.present_position + self.KP * self.err

            if l > self.robot.l_hip_x.angle_limit[0] and l < self.robot.l_hip_x.angle_limit[1] and r > self.robot.r_hip_x.angle_limit[0] and r < self.robot.r_hip_x.angle_limit[1]:
                self.lcommand = l
                self.rcommand = r

            else:
                # FIXME
                if l < self.robot.l_hip_x.angle_limit[0] or l > self.robot.l_hip_x.angle_limit[1]:
                    self.rcommand = self.robot.r_hip_x.present_position + \
                        2.0 * self.KP * self.err

                if r < self.robot.r_hip_x.angle_limit[0] or r > self.robot.r_hip_x.angle_limit[1]:
                    self.lcommand = self.robot.l_hip_x.present_position - \
                        2.0 * self.KP * self.err

            self.robot.l_hip_x.goal_position = self.lcommand
            self.robot.r_hip_x.goal_position = self.rcommand

            # print self.db, self.err, self.lcommand, self.rcommand

    def new_goal(self, dist):
        self.dist = dist


class SymmetricLegs(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.state = 'none'

    def update(self):

        if self.state == 'left':
            self.robot.r_hip_y.goal_position = - \
                self.robot.l_hip_y.present_position
            # self.robot.r_ankle_y.goal_position =- self.robot.l_ankle_y.present_position
            # self.robot.r_hip_x.goal_position =- self.robot.l_hip_x.present_position

        elif self.state == 'right':
            self.robot.l_hip_y.goal_position = - \
                self.robot.r_hip_y.present_position
            # self.robot.l_ankle_y.goal_position =- self.robot.r_ankle_y.present_position
            # self.robot.l_hip_x.goal_position =- self.robot.r_hip_x.present_position


class FootReact(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, lfoot, rfoot, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.lfoot = lfoot
        self.rfoot = rfoot

        # self.lthres = 0.5
        # self.rthres = 0.5

        self.lthres_up = 0.3
        self.rthres_up = 0.3

        self.lthres_down = 0.7
        self.rthres_down = 0.7

        self.lup = False
        self.rup = False

        self.ldown = True
        self.rdown = True

        self.ltrigup = False
        self.rtrigup = False

        self.lcooldown = 0.8
        self.rcooldown = 0.8
        self.ltime = 0
        self.rtime = 0

        self.init_done = False
        self.lmax = 0
        self.rmax = 0
        self.nb = 0

        self.prevosc = None
        self.losc = False
        self.rosc = False

    def update(self):

        if not self.init_done:
            self.lmax += self.lfoot.pressor_sum()
            self.rmax += self.rfoot.pressor_sum()
            self.nb += 1

            if self.nb == 10:
                self.lmax /= 10.0
                self.rmax /= 10.0
                self.init_done = True

        else:

            if((self.lfoot.pressor_sum() / self.lmax) >= self.lthres_down) and not self.ldown:
                self.ldown = True
                self.lup = False

            elif((self.lfoot.pressor_sum() / self.lmax) < self.lthres_up) and not self.lup:
                self.lup = True
                self.ldown = False

                if self.elapsed_time - self.ltime > self.lcooldown:
                    self.ltrigup = True
                    self.ltime = self.elapsed_time

            elif((self.lfoot.pressor_sum() / self.lmax) < self.lthres_up) and self.lup:
                self.ltrigup = False

            elif((self.rfoot.pressor_sum() / self.rmax) >= self.rthres_down) and not self.rdown:
                self.rdown = True
                self.rup = False

            elif((self.rfoot.pressor_sum() / self.rmax) < self.rthres_up) and not self.rup:
                self.rup = True
                self.rdown = False

                if self.elapsed_time - self.rtime > self.rcooldown:
                    self.rtrigup = True
                    self.rtime = self.elapsed_time

            elif((self.rfoot.pressor_sum() / self.rmax) < self.rthres_up) and self.rup:
                self.rtrigup = False

            # bof
            if self.ltrigup and (self.prevosc == 'RIGHT' or self.prevosc == None):
                self.losc = True
                self.rosc = False
                self.prevosc = 'LEFT'
                # print self.prevosc

            elif self.rtrigup and (self.prevosc == 'LEFT' or self.prevosc == None):
                self.rosc = True
                self.losc = False
                self.prevosc = 'RIGHT'
                # print self.prevosc

            else:
                self.rosc = False
                self.losc = False
