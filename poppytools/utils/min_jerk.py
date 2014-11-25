
from __future__ import division
import numpy as np

import collections

import pypot.primitive
import copy
import kinematics
# import time
import pypot.utils.pypot_time as time


class MJTraj():

    def __init__(self, initial, final, duration, init_vel=0.0,  init_acc=0.0,  final_vel=0.0, final_acc=0.0):
        self.a0 = initial
        self.a1 = init_vel
        self.a2 = init_acc / 2.0
        self.d5 = duration ** 5
        self.d4 = duration ** 4
        self.d3 = duration ** 3
        self.d2 = duration ** 2
        self.duration = duration
        self.durations = [0, duration]
        self.initial = initial
        self.final = final
        self.init_vel = init_vel
        self.final_vel = final_vel
        self.init_acc = init_acc
        self.final_acc = final_acc

        self.finals = [final]

        self.A = np.array([[self.d3, self.d4, self.d5], [
                          3 * self.d2, 4 * self.d3, 5 * self.d4], [6 * duration, 12 * self.d2, 20 * self.d3]])
        self.B = np.array([final - self.a0 - (self.a1 * duration) - (self.a2 * self.d2),
                           final_vel - self.a1 - (2 * self.a2 * duration), final_acc - (2 * self.a2)])

        self.X = []
        self.compute()

        self.other_gen = None

        self._mylambda = lambda x: self.a0 + self.a1 * x + self.a2 * x ** 2 + \
            self.X[0] * x ** 3 + self.X[1] * x ** 4 + self.X[2] * x ** 5

        self._generators = [self._mylambda]

    def compute(self):
        self.X = np.linalg.solve(self.A, self.B)

    def getValue(self, t):
        # FIXME: check time range
        # self.a0 + self.a1 * t + self.a2 * t ** 2 + self.X[0] * t ** 3 + self.X[1] * t ** 4 + self.X[2] * t ** 5
        return self._mygenerator[- 1](t)
        # return self._generator[ - 1](t)

    def domain(self, x):

        if not isinstance(x, collections.Iterable):
            x = np.array([x])

        domain = []
        for d in xrange(len(self.durations) - 1):
            d1 = []

            for xi in x:

                d1.append(
                    (xi >= self.durations[d]) & (xi < self.durations[d + 1]))

            domain.append(np.array(d1))
        return np.array(domain)

    def test_domain(self, x):
        return [((np.array(x) >= self.durations[i])) for i in xrange(len(self.durations) - 1)]

    def fix_input(self, x):
        if not isinstance(x, collections.Iterable):
            return np.array([0, x])
        else:
            return x

    def getGen(self):

        return lambda x: np.piecewise(x, self.domain(x), [self._generators[j] for j in xrange(len(self._generators))] + [self.finals[- 1]])

    def __add__(self,  othertraj):
        # print "ADD ", self.initial, self.final, othertraj.initial,
        # othertraj.final
        if othertraj.initial == self.final and othertraj.init_vel == self.init_vel and othertraj.init_acc == self.final_acc:
            self.final = othertraj.final
            self.final_vel = othertraj.final_vel
            self.final_acc = othertraj.final_acc
            self.other_gen = othertraj.getGen()

            tmpduration = self.duration
            # self.duration = self.duration + othertraj.duration

            # self._mygenerator(x) if filter(lambda y: y < self.duration, x) else self.other_gen(x)
            # self._mygenerator = lambda x: np.piecewise(np.array(x), [np.array(x) < self.duration, np.array(x) >= self.duration ], [self._mygenerator(x), self.other_gen(x)])
            l = len(self._generators) - 1
            l1 = len(self.durations) - 1
            l2 = len(othertraj.durations) - 1
            # self.durations.append(self.durations[l1] + othertraj.durations[l2])
            # self.durations.append(othertraj.durations[l2])
            # print "DEBUG ", len(self.durations), self.durations[l]

            # tmpgen = lambda x: self.other_gen(x - self.durations[l])
            # self._mygenerator.append(lambda x: np.piecewise(np.array(x), [np.array(x) < self.durations[l], np.array(x) >= self.durations[l]], [self._mygenerator[l], tmpgen]))

            tmpA = copy.deepcopy(self)
            tmpA.durations.append(self.durations[l1] + othertraj.durations[l2])
            tmpA.finals.append(othertraj.finals[- 1])

            tmpA._generators.append(
                lambda x: othertraj._mylambda(x - self.durations[- 1]))

            return tmpA

        else:
            print "Trajectories are not matching"
            return - 1


class MJLegs1D(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, mjtraj, ankle_offset=0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot
        self.mjtraj = mjtraj
        self.mj_gen = mjtraj.getGen()
        self.pos = 999999
        self.ankle_offset = ankle_offset
        self.l_hip_off = 0.0
        self.r_hip_off = 0.0

    def setup(self):
        self.t0 = time.time()

    def update(self):

        # print "elapsed", self.elapsed_time
        if self.elapsed_time <= self.mjtraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            self.pos = self.mj_gen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos
            hip, knee, ankle = kinematics.leg_up(self.pos)

            if self.leg == "left":
                # print '     LEFT UPDATE'
                self.robot.l_hip_y.goal_position = np.degrees(- hip)
                self.robot.l_knee_y.goal_position = np.degrees(- knee)
                self.robot.l_ankle_y.goal_position = np.degrees(
                    ankle) + self.ankle_offset

                # print self.robot.l_ankle_y.goal_position
                # self.robot.l_ankle_y.goal_position =  self.robot.l_hip_y.present_position

            elif self.leg == "right":
                # print '     RIGHT UPDATE'
                self.robot.r_hip_y.goal_position = np.degrees(- hip)
                self.robot.r_knee_y.goal_position = np.degrees(- knee)
                self.robot.r_ankle_y.goal_position = np.degrees(
                    ankle) + self.ankle_offset

                # self.robot.r_ankle_y.goal_position =  self.robot.r_hip_y.present_position

        elif self.elapsed_time > self.mjtraj.durations[- 1]:
            # print '   END', self.elapsed_time, self.mjtraj.durations[ - 1],
            # self.leg
            self.stop(wait=False)
            # self.running = False
            # pypot.primitive.LoopPrimitive.stop(self)
        else:
            print 'WTF??'


class MJLegs3D(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot

        self.up = 0.1
        self.duration = 0.0
        self.ztraj = None
        self.ytraj = None
        self.xtraj = None

        self.xgen = None
        self.ygen = None
        self.xgen = None

        self.x = 0.0
        self.y = 0.0
        # self.z = 0.0

        # self.mjtraj = mjtraj
        # self.mj_gen = mjtraj.getGen()
        # self.pos = 999999
        # self.ankle_offset = ankle_offset
        # self.l_hip_off = 0.0
        # self.r_hip_off = 0.0

    def add(self, x, y, duration):
        self.duration = duration

        self.ztraj = MJTraj(
            0, self.up, self.duration / 2.0) + MJTraj(self.up, 0, self.duration / 2.0)

        self.xtraj = MJTraj(self.x, x, self.duration)
        self.ytraj = MJTraj(self.y, y, self.duration)

        self.zgen = self.ztraj.getGen()
        self.ygen = self.ytraj.getGen()
        self.xgen = self.xtraj.getGen()

        self.x = x
        self.y = y
        # self.z = z

    def setup(self):
        self.t0 = time.time()

    def update(self):

        # print "elapsed", self.elapsed_time
        if self.elapsed_time <= self.ztraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            zpos = self.zgen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos

            xpos = self.xgen(self.elapsed_time)

            ypos = self.ygen(self.elapsed_time)

            theta = np.arcsin(xpos / kinematics.HMAX)

            phi = np.arcsin(ypos / kinematics.HMAX)

            hip, knee, ankle = kinematics.leg_up(zpos, theta)

            # print xpos, zpos, theta, np.degrees(theta), np.degrees(hip),
            # np.degrees(ankle),  self.robot.l_hip_y.goal_position

            if self.leg == "left":

                # print '     LEFT UPDATE'
                self.robot.l_hip_y.goal_position = np.degrees(- hip)
                self.robot.l_knee_y.goal_position = np.degrees(- knee)
                self.robot.l_hip_x.goal_position = np.degrees(phi)
                # self.robot.l_ankle_y.goal_position = np.degrees(ankle) #hum
                # bof

                # print self.robot.l_ankle_y.goal_position
                # self.robot.l_ankle_y.goal_position =  self.robot.l_hip_y.present_position

            elif self.leg == "right":

                # print '     RIGHT UPDATE'
                self.robot.r_hip_y.goal_position = np.degrees(- hip)
                self.robot.r_knee_y.goal_position = np.degrees(- knee)
                self.robot.r_hip_x.goal_position = np.degrees(phi)
                # self.robot.r_ankle_y.goal_position = np.degrees(ankle)

                # self.robot.r_ankle_y.goal_position =  self.robot.r_hip_y.present_position

        elif self.elapsed_time > self.ztraj.durations[- 1]:
            # print '   END', self.elapsed_time, self.mjtraj.durations[ - 1],
            # self.leg
            self.stop(wait=False)
            # self.running = False
            # pypot.primitive.LoopPrimitive.stop(self)
        else:
            print 'WTF??'


class MJLegs3DSym(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot

        self.up = 0.15
        self.duration = 0.0
        self.ztraj = None
        self.ytraj = None
        self.xtraj = None

        self.xgen = None
        self.ygen = None
        self.xgen = None

        self.x = 0.0
        self.y = 0.0
        # self.z = 0.0

        # self.mjtraj = mjtraj
        # self.mj_gen = mjtraj.getGen()
        # self.pos = 999999
        # self.ankle_offset = ankle_offset
        # self.l_hip_off = 0.0
        # self.r_hip_off = 0.0

        self.lhipy_off = 0
        self.lkneey_off = 0
        self.lhipx_off = 0
        self.lankley_off = 0

        self.rhipy_off = 0
        self.rkneey_off = 0
        self.rhipx_off = 0
        self.rankley_off = 0

    def add(self, x, y, duration):
        self.duration = duration

        self.ztraj = MJTraj(
            0, self.up, self.duration / 2.0) + MJTraj(self.up, 0, self.duration / 2.0)

        self.xtraj = MJTraj(self.x, x, self.duration / 2.0)
        self.ytraj = MJTraj(self.y, y, self.duration / 2.0)

        self.zgen = self.ztraj.getGen()
        self.ygen = self.ytraj.getGen()
        self.xgen = self.xtraj.getGen()

        self.x = x
        self.y = y
        # self.z = z

        self.lhipy_off = self.robot.l_hip_y.present_position
        # self.lkneey_off = self.robot.l_knee_y.present_position
        # self.lhipx_off = self.robot.l_hip_x.present_position
        # self.lankley_off = self.robot.l_ankle_y.present_position

        self.rhipy_off = self.robot.r_hip_y.present_position
        # self.rkneey_off = self.robot.r_knee_y.present_position
        # self.rhipx_off = self.robot.r_hip_x.present_position
        # self.rankley_off = self.robot.r_ankle_y.present_position

    def setup(self):
        self.t0 = time.time()

    def update(self):

        # print "elapsed", self.elapsed_time
        if self.elapsed_time <= self.ztraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            zpos = self.zgen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos

            theta = 0
            phi = 0
            if self.elapsed_time > self.ztraj.durations[- 1] / 2.0:

                xpos = self.xgen(self.elapsed_time)
                ypos = self.ygen(self.elapsed_time)
                theta = np.arcsin(xpos / kinematics.HMAX)
                phi = np.arcsin(ypos / kinematics.HMAX)

            hip, knee, ankle = kinematics.leg_up(zpos, theta)

            # print xpos, zpos, theta, np.degrees(theta), np.degrees(hip),
            # np.degrees(ankle),  self.robot.l_hip_y.goal_position

            if self.leg == "left":

                # print '     LEFT UPDATE'
                # self.robot.l_hip_y.goal_position = np.degrees( - hip)
                # self.robot.l_knee_y.goal_position = np.degrees( - knee)
                # self.robot.l_hip_x.goal_position = np.degrees( phi)
                # self.robot.l_ankle_y.goal_position = np.degrees(ankle) #hum
                # bof

                # self.robot.r_hip_y.goal_position = np.degrees( theta)
                # self.robot.r_hip_x.goal_position = np.degrees( - phi)

                # print '     LEFT UPDATE'
                self.robot.l_hip_y.goal_position = self.lhipy_off + \
                    np.degrees(- hip)
                self.robot.l_knee_y.goal_position = self.lkneey_off + \
                    np.degrees(- knee)
                self.robot.l_hip_x.goal_position = self.lhipx_off + \
                    np.degrees(phi)
                self.robot.l_ankle_y.goal_position = self.lankley_off + \
                    np.degrees(ankle)  # hum bof

                self.robot.r_hip_y.goal_position = np.degrees(theta)
                self.robot.r_hip_x.goal_position = np.degrees(- phi)

                # print self.robot.l_ankle_y.goal_position
                # self.robot.l_ankle_y.goal_position =  self.robot.l_hip_y.present_position

            elif self.leg == "right":

                # print '     RIGHT UPDATE'
                self.robot.r_hip_y.goal_position = self.rhipy_off + \
                    np.degrees(- hip)
                self.robot.r_knee_y.goal_position = self.rkneey_off + \
                    np.degrees(- knee)
                self.robot.r_hip_x.goal_position = self.rhipx_off + \
                    np.degrees(phi)
                self.robot.r_ankle_y.goal_position = self.rankley_off + \
                    np.degrees(ankle)

                self.robot.l_hip_y.goal_position = np.degrees(theta)
                self.robot.l_hip_x.goal_position = np.degrees(- phi)

                # self.robot.r_ankle_y.goal_position =  self.robot.r_hip_y.present_position

        elif self.elapsed_time > self.ztraj.durations[- 1]:
            # print '   END', self.elapsed_time, self.mjtraj.durations[ - 1],
            # self.leg
            self.stop(wait=False)
            # self.running = False
            # pypot.primitive.LoopPrimitive.stop(self)
        else:
            print 'WTF??'


class MJLegsUp3D(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, up=0.1, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot

        self.up = up
        self.duration = 0.0
        self.ztraj = None
        self.ytraj = None
        self.xtraj = None

        self.xgen = None
        self.ygen = None
        self.xgen = None

        self.x = 0.0
        self.y = 0.0
        # self.z = 0.0

        # self.mjtraj = mjtraj
        # self.mj_gen = mjtraj.getGen()
        # self.pos = 999999
        # self.ankle_offset = ankle_offset
        # self.l_hip_off = 0.0
        # self.r_hip_off = 0.0

        self.lhipy_off = 0
        self.lkneey_off = 0
        self.lhipx_off = 0
        self.lankley_off = 0

        self.rhipy_off = 0
        self.rkneey_off = 0
        self.rhipx_off = 0
        self.rankley_off = 0

    def add(self, x, y, duration):
        self.duration = duration

        self.ztraj = MJTraj(0, self.up, self.duration)

        self.lhipy_off = self.robot.l_hip_y.present_position
        # self.lkneey_off = self.robot.l_knee_y.present_position
        self.lhipx_off = self.robot.l_hip_x.present_position
        # self.lankley_off = self.robot.l_ankle_y.present_position

        self.rhipy_off = self.robot.r_hip_y.present_position
        # self.rkneey_off = self.robot.r_knee_y.present_position
        self.rhipx_off = self.robot.r_hip_x.present_position
        # self.rankley_off = self.robot.r_ankle_y.present_position

        if self.leg == 'left':

            self.xtraj = MJTraj(self.rhipx_off, x, self.duration)
            self.ytraj = MJTraj(self.rhipy_off, y, self.duration)

        elif self.leg == 'right':

            self.xtraj = MJTraj(self.lhipx_off, x, self.duration)
            self.ytraj = MJTraj(self.lhipy_off, y, self.duration)

        self.zgen = self.ztraj.getGen()
        self.ygen = self.ytraj.getGen()
        self.xgen = self.xtraj.getGen()

        self.x = x
        self.y = y
        # self.z = z

    def setup(self):
        self.t0 = time.time()

    def update(self):

        # print "elapsed", self.elapsed_time
        if self.elapsed_time <= self.ztraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            zpos = self.zgen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos

            theta = 0
            phi = 0
            # if self.elapsed_time > self.ztraj.durations[ - 1] / 2.0:

            xpos = self.xgen(self.elapsed_time)
            ypos = self.ygen(self.elapsed_time)
            theta = np.arcsin(xpos / kinematics.HMAX)
            phi = np.arcsin(ypos / kinematics.HMAX)

            if np.isnan(theta):
                theta = 0.0
            if np.isnan(phi):
                phi = 0.

            hip, knee, ankle = kinematics.leg_up(zpos, 0)

            # print xpos, zpos, theta, np.degrees(theta), np.degrees(hip),
            # np.degrees(ankle),  self.robot.l_hip_y.goal_position

            if self.leg == "left":

                # print '     LEFT UPDATE'
                # self.robot.l_hip_y.goal_position = np.degrees( - hip)
                # self.robot.l_knee_y.goal_position = np.degrees( - knee)
                # self.robot.l_hip_x.goal_position = np.degrees( phi)
                # self.robot.l_ankle_y.goal_position = np.degrees(ankle) #hum
                # bof

                # self.robot.r_hip_y.goal_position = np.degrees( theta)
                # self.robot.r_hip_x.goal_position = np.degrees( - phi)

                # print '     LEFT UPDATE'
                self.robot.l_hip_y.goal_position = self.lhipy_off + \
                    np.degrees(- hip)
                self.robot.l_knee_y.goal_position = self.lkneey_off + \
                    np.degrees(- knee)
                self.robot.l_hip_x.goal_position = self.lhipx_off + \
                    np.degrees(phi)
                self.robot.l_ankle_y.goal_position = self.lankley_off + \
                    np.degrees(ankle)  # hum bof

                #??
                # self.robot.r_hip_y.goal_position = np.degrees(theta)
                # self.robot.r_hip_x.goal_position = np.degrees(- phi)
                # self.robot.r_ankle_y.goal_position = np.degrees(theta)

                # self.robot.r_hip_y.goal_position = 0.0
                # self.robot.r_hip_x.goal_position = 0.0

                # print self.robot.l_ankle_y.goal_position
                # self.robot.l_ankle_y.goal_position =  self.robot.l_hip_y.present_position

            elif self.leg == "right":

                # print '     RIGHT UPDATE'
                self.robot.r_hip_y.goal_position = self.rhipy_off + \
                    np.degrees(- hip)
                self.robot.r_knee_y.goal_position = self.rkneey_off + \
                    np.degrees(- knee)
                self.robot.r_hip_x.goal_position = self.rhipx_off + \
                    np.degrees(phi)
                self.robot.r_ankle_y.goal_position = self.rankley_off + \
                    np.degrees(ankle)

                #??
                # self.robot.l_hip_y.goal_position = 0.0
                # self.robot.l_hip_x.goal_position = 0.0

                # self.robot.l_hip_y.goal_position = np.degrees(theta)
                # self.robot.l_hip_x.goal_position = np.degrees(- phi)
                # self.robot.l_ankle_y.goal_position = np.degrees(theta)

                # self.robot.r_ankle_y.goal_position =  self.robot.r_hip_y.present_position

        elif self.elapsed_time > self.ztraj.durations[- 1]:
            # print '   END', self.elapsed_time, self.mjtraj.durations[ - 1],
            # self.leg
            self.stop(wait=False)
            # self.running = False
            # pypot.primitive.LoopPrimitive.stop(self)
        else:
            print 'WTF??'


class MJLegsDown3D(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, up=0.1, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot

        self.up = up
        self.duration = 0.0
        self.ztraj = None
        self.ytraj = None
        self.xtraj = None

        self.xgen = None
        self.ygen = None
        self.xgen = None

        self.x = 0.0
        self.y = 0.0
        # self.z = 0.0

        # self.mjtraj = mjtraj
        # self.mj_gen = mjtraj.getGen()
        # self.pos = 999999
        # self.ankle_offset = ankle_offset
        # self.l_hip_off = 0.0
        # self.r_hip_off = 0.0

        self.lhipy_off = 0
        self.lkneey_off = 0
        self.lhipx_off = 0
        self.lankley_off = 0

        self.rhipy_off = 0
        self.rkneey_off = 0
        self.rhipx_off = 0
        self.rankley_off = 0

    def add(self, x, y, duration):
        self.duration = duration

        self.ztraj = MJTraj(self.up, 0, self.duration)

        self.xtraj = MJTraj(self.x, x / 2., self.duration)
        self.ytraj = MJTraj(self.y, y / 2., self.duration)

        self.zgen = self.ztraj.getGen()
        self.ygen = self.ytraj.getGen()
        self.xgen = self.xtraj.getGen()

        self.x = x / 2.
        self.y = y / 2.
        # self.z = z

        # self.lhipy_off = self.robot.l_hip_y.present_position
        # self.lkneey_off = self.robot.l_knee_y.present_position
        # self.lhipx_off = self.robot.l_hip_x.present_position
        # self.lankley_off = self.robot.l_ankle_y.present_position

        # self.rhipy_off = self.robot.r_hip_y.present_position
        # self.rkneey_off = self.robot.r_knee_y.present_position
        # self.rhipx_off = self.robot.r_hip_x.present_position
        # self.rankley_off = self.robot.r_ankle_y.present_position

    def setup(self):
        self.t0 = time.time()

    def update(self):

        # print "elapsed", self.elapsed_time
        if self.elapsed_time <= self.ztraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            zpos = self.zgen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos

            xpos = self.xgen(self.elapsed_time)
            ypos = self.ygen(self.elapsed_time)
            theta = np.arcsin(xpos / kinematics.HMAX)
            phi = np.arcsin(ypos / kinematics.HMAX)

            hip, knee, ankle = kinematics.leg_up(zpos, theta)

            # print xpos, zpos, theta, np.degrees(theta), np.degrees(hip),
            # np.degrees(ankle),  self.robot.l_hip_y.goal_position

            if self.leg == "left":

                # print '     LEFT UPDATE'
                # self.robot.l_hip_y.goal_position = np.degrees( - hip)
                # self.robot.l_knee_y.goal_position = np.degrees( - knee)
                # self.robot.l_hip_x.goal_position = np.degrees( phi)
                # self.robot.l_ankle_y.goal_position = np.degrees(ankle) #hum
                # bof

                # self.robot.r_hip_y.goal_position = np.degrees( theta)
                # self.robot.r_hip_x.goal_position = np.degrees( - phi)

                # print '     LEFT UPDATE'
                self.robot.l_hip_y.goal_position = self.lhipy_off + \
                    np.degrees(- hip)
                self.robot.l_knee_y.goal_position = self.lkneey_off + \
                    np.degrees(- knee)
                self.robot.l_hip_x.goal_position = self.lhipx_off + \
                    np.degrees(phi)
                self.robot.l_ankle_y.goal_position = self.lankley_off + \
                    np.degrees(ankle)  # hum bof

                self.robot.r_hip_y.goal_position = np.degrees(theta)
                self.robot.r_hip_x.goal_position = np.degrees(- phi)

                # print self.robot.l_ankle_y.goal_position
                # self.robot.l_ankle_y.goal_position =  self.robot.l_hip_y.present_position

            elif self.leg == "right":

                # print '     RIGHT UPDATE'
                self.robot.r_hip_y.goal_position = self.rhipy_off + \
                    np.degrees(- hip)
                self.robot.r_knee_y.goal_position = self.rkneey_off + \
                    np.degrees(- knee)
                self.robot.r_hip_x.goal_position = self.rhipx_off + \
                    np.degrees(phi)
                self.robot.r_ankle_y.goal_position = self.rankley_off + \
                    np.degrees(ankle)

                self.robot.l_hip_y.goal_position = np.degrees(theta)
                self.robot.l_hip_x.goal_position = np.degrees(- phi)

                # self.robot.r_ankle_y.goal_position =  self.robot.r_hip_y.present_position

        elif self.elapsed_time > self.ztraj.durations[- 1]:
            # print '   END', self.elapsed_time, self.mjtraj.durations[ - 1],
            # self.leg
            self.stop(wait=False)
            # self.running = False
            # pypot.primitive.LoopPrimitive.stop(self)
        else:
            print 'WTF??'


class MJLegs1DSym(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, leg, mjtraj, ankle_offset=0, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.leg = leg
        # self.robot = robot
        self.mjtraj = mjtraj
        self.mj_gen = mjtraj.getGen()
        self.pos = 999999
        self.ankle_offset = ankle_offset
        self.l_hip_off = 0.0
        self.r_hip_off = 0.0

    def update(self):

        if self.elapsed_time <= self.mjtraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            self.pos = self.mj_gen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos
            hip, knee, ankle = kinematics.leg_up(self.pos)

            self.l_hip_off = self.robot.l_hip_y.present_position
            self.r_hip_off = self.robot.r_hip_y.present_position

            if self.leg == "left":

                # - self.robot.r_hip_y.present_position
                self.robot.l_hip_y.goal_position = np.degrees(
                    - hip) + self.l_hip_off
                self.robot.l_knee_y.goal_position = np.degrees(- knee)
                self.robot.l_ankle_y.goal_position = np.degrees(
                    ankle) + self.ankle_offset

            elif self.leg == "right":

                # - self.robot.l_hip_y.present_position
                self.robot.r_hip_y.goal_position = np.degrees(
                    - hip) + self.r_hip_off
                self.robot.r_knee_y.goal_position = np.degrees(- knee)
                self.robot.r_ankle_y.goal_position = np.degrees(
                    ankle) + self.ankle_offset
        else:
            self.stop()


class MJArm(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, arm, goal, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.arm = arm
        self.goal = goal  # a tuple

        self.L1 = kinematics.L_ARM
        self.L2 = kinematics.L_FOREARM
        self.HMAX = self.L1 + self.L2 - 0.001

    def update(self):

        if self.elapsed_time <= self.mjtraj.durations[- 1]:
            # if abs(self.pos - self.mjtraj.final) > 0.001:

            self.pos = self.mj_gen(self.elapsed_time)
            # pos = self.mjtraj([self.elapsed_time])
            # print pos
            hip, knee, ankle = kinematics.leg_up(self.pos)

            if self.leg == "left":

                self.robot.l_hip_y.goal_position = np.degrees(- hip)
                self.robot.l_knee_y.goal_position = np.degrees(- knee)
                self.robot.l_ankle_y.goal_position = np.degrees(ankle)

            elif self.leg == "right":

                self.robot.r_hip_y.goal_position = np.degrees(- hip)
                self.robot.r_knee_y.goal_position = np.degrees(- knee)
                self.robot.r_ankle_y.goal_position = np.degrees(ankle)
        else:
            self.stop()


class MoveGprojMJ(pypot.primitive.LoopPrimitive):
    # FIXME

    def __init__(self, robot, cx, duration,  ctrlabs=True, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.mj = None
        self.db = None
        self.theta2 = None
        self.C = None
        self.cx = cx
        self.c_goal = None
        self.duration = duration
        self.ctrlabs = ctrlabs
        self.mjstarted = None
        self.add(cx, duration)

    def init_traj(self):
        # compute the FK from hip angles (assuming straight legs, flat ground
        # and rigidity)

        self.db, t22 = kinematics.get_db(self.robot)

        # self.theta2 = self.robot.r_ankle_x.present_position + 90.0 - kinematics.HIP_OFF
        # self.theta2 = 90.0 - self.robot.r_ankle_x.present_position- 4.0

        # ok get theta2 from hip configuration
        self.theta2 = - t22

        # la = self.robot.l_ankle_x.present_position + 90.0 + 4.0
        la = 90 - self.robot.l_ankle_x.present_position - \
            kinematics.HIP_OFF
        rh = self.robot.r_hip_x.present_position - \
            90.0 - kinematics.HIP_OFF
        lh = self.robot.l_hip_x.present_position + \
            90.0 + kinematics.HIP_OFF

        th = 360.0 - la + rh - lh

        C, t1, t2, t3, t4 = kinematics.get_gproj(
            np.radians(self.theta2), self.db)
        # C,t1,t2,t3,t4=kinematics.get_gproj(np.radians((self.theta2 + th) / 2.0),self.db)
        # C,t1,t2,t3,t4=kinematics.get_gproj(np.radians(th),self.db)

        self.C = C
        self.c_goal = self.cx + self.db / 2.0

        # print self.theta2, th, la, rh, lh, self.C, self.db,
        # self.robot.r_ankle_x.present_position,
        # self.robot.l_ankle_x.present_position

        self.mj = MJTraj(self.C, self.c_goal, self.duration).getGen()
        self.mjstarted = True

    def add(self, cx, duration):
        self.cx = cx
        self.duration = duration
        # print self.mjstarted
        self.mjstarted = False

    def update(self):
        if not self.mjstarted:
            self.init_traj()
        else:
            if self.elapsed_time <= self.duration:

                c, t1, t2, t3, t4, t5 = kinematics.get_inv_gproj(
                    self.mj(self.elapsed_time), self.db)

                # print kinematics.get_db(self.robot), self.theta2, self.C,
                # self.c_goal, c, np.degrees(t1), np.degrees(t2),
                # np.degrees(t3), np.degrees(t4), np.degrees(t5)

                self.robot.l_ankle_x.goal_position = np.degrees(
                    t2 - np.pi / 2.0) - kinematics.HIP_OFF
                self.robot.r_ankle_x.goal_position = np.degrees(
                    t1 - np.pi / 2.0) + kinematics.HIP_OFF
                self.robot.l_hip_x.goal_position = np.degrees(
                    t4 - np.pi / 2.0) - kinematics.HIP_OFF
                self.robot.r_hip_x.goal_position = np.degrees(
                    np.pi / 2.0 - t3) + kinematics.HIP_OFF

                if self.ctrlabs:
                    self.robot.abs_x.goal_position = np.degrees(- t5)

            else:
                del self.mj
                # self.stop()
                self.stop(wait=False)


class goto_mj(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, motors, duration, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.add(motors, duration)

    def update(self):
        if not self.mjstarted:
            self.init_traj()
        else:
            if self.elapsed_time <= self.duration:
                # if abs(self.pos - self.mjtraj.final) > 0.001:
                for motorname, mj in self.trajs.iteritems():
                    getattr(self.robot, motorname).goal_position = mj(
                        self.elapsed_time)
            else:
                del self.trajs
                self.stop()

    def add(self, motors, duration):
        self.motors = motors
        self.duration = duration
        self.trajs = {}
        self.mjstarted = False

    def init_traj(self):
        self.trajs = {m: MJTraj(getattr(self.robot, m).present_position,  g, self.duration).getGen(
        ) for m, g in self.motors.iteritems()}
        self.mjstarted = True


class goto_mjtraj(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, motors_traj, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)
        self.add(motors_traj)

    def update(self):
        if not self.mjstarted:
            self.init_traj()
        else:
            if np.array(self.elapsed_time <= np.array(self.durations)).all():
                # if abs(self.pos - self.mjtraj.final) > 0.001:
                for motorname, mj in self.motors_traj.iteritems():
                    getattr(self.robot, motorname).goal_position = mj(
                        self.elapsed_time)
            else:
                # FIXME
                self.stop()

    def add(self, motors):
        self.motors_traj = {m: mj.getGen() for m, mj in motors.iteritems()}
        self.durations = [d.durations[- 1] for d in motors.values()]
        self.mjstarted = False

    def init_traj(self):
        self.mjstarted = True


def yes_no(robot, s, wait=False):

    if s == 'y':
        mj1 = MJTraj(robot.head_y.present_position, 15, 0.25)
        mj2 = MJTraj(15, - 15, 0.4)
        mj3 = MJTraj(- 15, 0, 0.25)

        robot.attach_primitive(
            goto_mjtraj(robot, {'head_y': mj1 + mj2 + mj3}), 'yes')
        robot.yes.start()
        if wait:
            robot.yes.wait_to_stop()

    elif s == 'n':
        mj1 = MJTraj(robot.head_z.present_position, 15, 0.25)
        mj2 = MJTraj(15, - 15, 0.4)
        mj3 = MJTraj(- 15, 0, 0.25)

        robot.attach_primitive(
            goto_mjtraj(robot, {'head_z': mj1 + mj2 + mj3}), 'no')
        robot.no.start()
        if wait:
            robot.no.wait_to_stop()


class FeetReflex(pypot.primitive.LoopPrimitive):

    def __init__(self, robot, lfoot, rfoot, refresh_freq=50):
        pypot.primitive.LoopPrimitive.__init__(self, robot, refresh_freq)

        self.robot.attach_primitive(
            kinematics.FootReact(self.robot, lfoot, rfoot), 'footreact')
        self.robot.footreact.start()

        self.LEG_TIME = 0.25

        self.up_duration = self.LEG_TIME / 2.0
        self.down_duration = self.LEG_TIME / 2.0

        # self.up = 0.003
        self.up = 0.005
        # up = 0.00

        # duration = 2.0
        # up = 0.05

        self.mj1 = MJTraj(0, self.up, self.up_duration)
        self.mj2 = MJTraj(self.up, 0, self.down_duration)

        self.mj3 = self.mj1 + self.mj2

        # self.mjf1 = MJTraj(1, 2, self.up_duration)
        # self.mjf2 = MJTraj(2, 1, self.down_duration)

        self.mjf1 = MJTraj(0, self.up, self.up_duration)
        self.mjf2 = MJTraj(self.up, 0, self.down_duration)

        self.mjf3 = self.mjf1 + self.mjf2

        self.robot.attach_primitive(
            MJLegs1D(self.robot, "left", self.mj1, 1), 'mjleftup')
        self.robot.attach_primitive(
            MJLegs1D(self.robot, "left", self.mj2, 1), 'mjleftdown')

        self.robot.attach_primitive(
            MJLegs1D(self.robot, "right", self.mj1, 1), 'mjrightup')
        self.robot.attach_primitive(
            MJLegs1D(self.robot, "right", self.mj2, 1), 'mjrightdown')

        self.robot.attach_primitive(
            MJLegs1D(self.robot, "left", self.mj3, 0), 'mjleftupdown')
        self.robot.attach_primitive(
            MJLegs1D(self.robot, "right", self.mj3, 0), 'mjrightupdown')

        # self.robot.attach_primitive(goto_mjtraj(self.robot, {'l_ankle_y': self.mjf3}), 'mjleftupdown')
        # self.robot.attach_primitive(goto_mjtraj(self.robot, {'r_ankle_y': self.mjf3}), 'mjrightupdown')

        # self.X_COM = 0.028
        # self.FREQ = 1.116

        self.X_COM = 0
        self.FREQ = 0

        self.robot.attach_primitive(
            kinematics.MoveGprojOSC(robot, self.X_COM, self.FREQ), 'gsin')
        # self.robot.attach_primitive(kinematics.MoveGprojSin(robot, self.X_COM, self.FREQ), 'gsin')
        self.robot.gsin.start()

    def update(self):

        # print self.robot.gsin.right, self.robot.gsin.left
        # if self.robot.footreact.rosc:
        if self.robot.gsin.right:
            print '\tRIGHT UP'
            # self.robot.goto_position({'l_hip_y': 3.0,'r_hip_y': 0.0 }, 0.1, wait = False)

            # self.robot.mjrightupdown.start()
            self.robot.mjleftupdown.start()
            # nb = nb - 1

        # elif self.robot.footreact.losc:
        elif self.robot.gsin.left:
            print '\tLEFT UP'

            # self.robot.mjleftupdown.start()
            self.robot.mjrightupdown.start()
        # else:
        #     time.sleep(0.02)


if __name__ == '__main__':

    ts = np.arange(0, 2.01, .01)

    m1 = MJTraj(0, 10, 0.5)
    m2 = MJTraj(10, 15, 0.5)
    m3 = MJTraj(15, 20, 0.5)
    m4 = m1 + m2 + m3 + MJTraj(20, 25, 0.5)

    # print m4.initial, m4.final, m4.duration,  m4.durations

    m1t = m1.getGen()
    m2t = m2.getGen()
    m3t = m3.getGen()
    m4t = m4.getGen()

    # print m1t(ts)
    # print m2t(ts)
    # print m3t(ts)
    # print len(m4._generators)

    m1 = MJTraj(0, 10, 0.5)
    m2 = MJTraj(10, 15, 0.5)
    m4 = m1 + m2 + m3
    # m4 =  MJTraj(0, 10, 2.0)
    m4t = m4.getGen()

    t = 0.0
    ta = []
    print
    for i in range(20):
        # print m4t(np.ones(10) * t)
        ta.append(t)
        print m4.domain(t)
        # print m4._generators
        # print m4.test_domain(t)
        print t, m4t(t)

        t += 0.10

    ta = np.array(ta)
    # print m4.domain(1.9)
    print

    print ta
    print m4t(ta)
    print m4.domain(ta)
    print m4t(np.asarray([0.4, 0.9, 1.4, 1.9, 2.4]))

    print m4.duration
    # print m4.test_domain(ta)
    # print m4.domain(ta)

    # print m4t(ts)

    # stop

    # plt.plot(ts, m1t(ts))
    # plt.plot(ts, m2t(ts))
    # plt.plot(ts, m4t(ts))

    # plt.plot(ts, mmm(ts))
    # m = simpyMJ(0.0, 10.0, 0.5, final_vel = 5.0)
    # m = npMJ(0.0, 10.0, 0.5, final_vel = 5.0)
    # plt.plot(ts, m(ts))
    # plt.plot(kinematics.deriv(jerk))
    # plt.plot(jerk)
    # plt.plot(test)
    # plt.show()
