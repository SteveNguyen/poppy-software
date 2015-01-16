#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: 'qm_learn_stab.py'
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen@inria.fr
#  Created	: Wednesday, November 26 2014
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#

import random
import numpy as np
import math
import time as systime
import pypot.utils.pypot_time as time
import sys


from pypot.vrep import from_vrep

import pypot.vrep.remoteApiBindings.vrep as vrep

# from poppytools.configuration.config import poppy_config

import poppytools.utils.kinematics as kinematics
import poppytools.utils.min_jerk as min_jerk

import pypot

from pyvrep.xp import PoppyVrepXp

# Quasimetric stuffs
from pyqmc.ProbUtils import Distrib, CondDistrib, Transition, totuple
from pyqmc.QMLib import QM

# Parameters
_P_THRES = 0.001

# _P_THRES=0.00

XMIN = -0.1
XMAX = 0.3

XCARD = 11

YMIN = -0.1
YMAX = 0.3

YCARD = 11

DXMIN = -1.5
DXMAX = 1.5

DXCARD = 11

DYMIN = -1.
DYMAX = 1.

DYCARD = 11

UXMIN = - 0.2
UXMAX = 0.2

UXCARD = 11

UYMIN = - 0.3
UYMAX = -0.1

UYCARD = 11

# SIGMA_POS=0.2
# SIGMA_VEL=0.2

SIGMA_POS = 0.1
SIGMA_VEL = 0.1


# replay data from file

def cost(x, u):
    return 1.0


def online_from_file_ind(filename, qmx, qmy, Xpos, Ypos, Xvel, Yvel, Ux, Uy):
    fd = open(filename, 'r')
    data = []
    for l in fd:
        if not l[0] == '#':
            l = l.replace('\n', '').split(' ')
            # print l

            d = np.array([float(d) for d in l if d != ''])
            data.append(d)

            # xy = XY.Discretize((d[0], d[1], d[2], d[3]))
            # u = U.Discretize((d[4], d[5]))
            # next_xy = XY.Discretize((d[6], d[7], d[8], d[9]))

            xpos = Xpos.Discretize(d[0])
            ypos = Ypos.Discretize(d[1])
            xvel = Xvel.Discretize(d[2])
            yvel = Yvel.Discretize(d[3])

            ux = Ux.Discretize(d[4])
            uy = Uy.Discretize(d[5])

            n_xpos = Xpos.Discretize(d[6])
            n_ypos = Ypos.Discretize(d[7])
            n_xvel = Xvel.Discretize(d[8])
            n_yvel = Yvel.Discretize(d[9])

            qmx.OnlineUpdate(
                (xpos, xvel), (n_xpos, n_xvel), (ux, ), cost((), ()))
            qmy.OnlineUpdate(
                (ypos, yvel), (n_ypos, n_yvel), (uy, ), cost((), ()))


class Stab(PoppyVrepXp):

    def Init(self):

        self.robot._primitive_manager._filter = np.sum

        # self.quasix = quasix
        # self.quasiy = quasiy

        # self.Xpos = Xpos
        # self.Ypos = Ypos
        # self.Xvel = Xvel
        # self.Yvel = Yvel
        # self.Ux = Ux
        # self.Uy = Uy
        # self.X = X
        # self.Y = Y

        self.set_goal(0, 0.05, 0, 0)
        self.fall = False

        self.nb_reset = 0

        self.transitions_log = open(
            'onlineqm_trans_' + str(self.port) + '.dat', 'a+')
        self.transitions_logd = open(
            'onlineqm_transd_' + str(self.port) + '.dat', 'a+')

        self.state = 'right'
        self.goal_leg = 0

        self.duration = 0.25

        self.up = 0.1

        self.vrepio = self.robot._controllers[0].io

        self.obj_h = {}

        objects = self.vrepio.call_remote_api('simxGetObjectGroupData',
                                              vrep.sim_object_shape_type, 0)
        self.obj_h = dict(zip(objects[3], objects[0]))

        self.joint_h = {}

        joints = self.vrepio.call_remote_api('simxGetObjectGroupData',
                                             vrep.sim_object_joint_type, 0)

        self.joint_h = dict(zip(joints[3], joints[0]))

        self.dummies_h = {}

        dummies = self.vrepio.call_remote_api('simxGetObjectGroupData',
                                              vrep.sim_object_dummy_type, 0)

        self.dummies_h = dict(zip(dummies[3], dummies[0]))

        self.lfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
            'foot_left_visual'], self.dummies_h['CoM'], streaming=True)

        self.rfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
            'foot_right_visual'], self.dummies_h['CoM'], streaming=True)

        self.com_abs = self.vrepio.call_remote_api(
            'simxGetObjectPosition', self.dummies_h['CoM'], - 1, streaming=True)

        self.com_vel = self.vrepio.call_remote_api(
            'simxGetObjectVelocity', self.dummies_h['CoM'], streaming=True)

        self.lfoot = np.array(
            [self.lfoot[0], self.lfoot[1], self.lfoot[2]])
        self.rfoot = np.array(
            [self.rfoot[0], self.rfoot[1], self.rfoot[2]])
        self.com = np.array(
            [self.com_abs[0], self.com_abs[1], self.com_abs[2]])

        self.lfoot_pos_com = np.array(self.lfoot)
        self.rfoot_pos_com = np.array(self.rfoot)

        self.lfoot_vel_com = np.array([0, 0, 0])
        self.rfoot_vel_com = np.array([0, 0, 0])

        self.robot.attach_primitive(
            min_jerk.MJLegsUp3D(self.robot, 'left', self.up), 'lleg3d_up')
        self.robot.attach_primitive(
            min_jerk.MJLegsUp3D(self.robot, 'right', self.up), 'rleg3d_up')

        self.robot.attach_primitive(
            min_jerk.MJLegsDown3D(self.robot, 'left', self.up), 'lleg3d_down')
        self.robot.attach_primitive(
            min_jerk.MJLegsDown3D(self.robot, 'right', self.up), 'rleg3d_down')

        self.t0 = time.time()
        # time.sleep(0.1)

        self.update()
        self.current_state = (self.rfoot_pos_com[0], -self.rfoot_pos_com[
            1], self.rfoot_vel_com[0], -self.rfoot_vel_com[1])
        self.current_cmd = None

    def set_goal(self, xpos, ypos, xvel, yvel):

        xpos = self.Xpos.Discretize(0.0)
        ypos = self.Ypos.Discretize(0.05)
        xvel = self.Xvel.Discretize(0.0)
        yvel = self.Yvel.Discretize(0.0)

        goalx = (xpos, xvel)
        goaly = (ypos, yvel)

        self.goal_vertx = quasix.Vertices[totuple(goalx)]
        self.goal_verty = quasiy.Vertices[totuple(goaly)]

    def update(self):

        self.lfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
            'foot_left_visual'], self.dummies_h['CoM'], streaming=True)

        self.rfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
            'foot_right_visual'], self.dummies_h['CoM'], streaming=True)

        self.com_abs = self.vrepio.call_remote_api(
            'simxGetObjectPosition', self.dummies_h['CoM'], - 1, streaming=True)

        self.com_vel = self.vrepio.call_remote_api(
            'simxGetObjectVelocity', self.dummies_h['CoM'], streaming=True)

        self.lfoot = np.array(
            [self.lfoot[0], self.lfoot[1], self.lfoot[2]])
        self.rfoot = np.array(
            [self.rfoot[0], self.rfoot[1], self.rfoot[2]])
        self.com = np.array(
            [self.com_abs[0], self.com_abs[1], self.com_abs[2]])

        lfoot_pos_com = np.array(self.lfoot)
        rfoot_pos_com = np.array(self.rfoot)

        dt = time.time() - self.t0
        self.t0 = time.time()

        if dt <= 0.0:
            self.lfoot_vel_com = np.array([0, 0, 0])
            self.rfoot_vel_com = np.array([0, 0, 0])
        else:

            self.lfoot_vel_com = (
                np.array(lfoot_pos_com) - np.array(self.lfoot_pos_com)) / dt
            self.rfoot_vel_com = (
                np.array(rfoot_pos_com) - np.array(self.rfoot_pos_com)) / dt

        self.lfoot_pos_com = lfoot_pos_com
        self.rfoot_pos_com = rfoot_pos_com

        if self.com_abs[2] < 0.3:
            print 'FALL:', self.state
            self.fall = True

            self.robot.reset_simulation()
            print 'resetting'
            self.nb_reset += 1

            if self.nb_reset == 5:

                print self.quasix.Compute(self.goal_vertx)
                print
                print self.quasiy.Compute(self.goal_verty)
                print self.quasix.ComputePolicy()
                print
                print self.quasiy.ComputePolicy()
                self.nb_reset = 0

        else:

            if self.robot.CollisionLFoot.colliding and not self.robot.CollisionRFoot.colliding:
                # Only LFoot on ground
                self.state = 'right'

            elif not self.robot.CollisionLFoot.colliding and self.robot.CollisionRFoot.colliding:
                # only RFoot on ground
                self.state = 'left'

            elif self.robot.CollisionLFoot.colliding and self.robot.CollisionRFoot.colliding:
                # Both Feet on ground. Take the decision here.
                self.state = 'both'

            else:
                # gnin?
                print 'FLY'
                self.state = 'fly'

                # poppy.reset_simulation()
                # print "resetting"

    def run(self):
        self.Init()

        while True:
            self.update()

            if self.state == 'right':
                # time.sleep(0.05)
                # print 'FEET', self.lfoot_pos_com, self.rfoot_pos_com
                pass

            elif self.state == 'left':
                # time.sleep(0.05)
                # print 'FEET', self.lfoot_pos_com, self.rfoot_pos_com
                pass
            elif self.state == 'both':
                print 'FEET', self.lfoot_pos_com, self.rfoot_pos_com

                if self.goal_leg == 1:
                    # previous was left
                    leg = 'right'
                    self.goal_leg = 0
                    self.goal_x = 0
                    self.goal_y = 0
                    print leg
                    self.move_up(leg)

                    state = (self.lfoot_pos_com[0], self.lfoot_pos_com[
                        1], self.lfoot_vel_com[0], self.lfoot_vel_com[1])

                    x, y = self.get_goal(state)
                    # get the goal
                    self.current_state = state
                    self.current_cmd = (x, y)

                    self.move_down(leg, x, y)

                elif self.goal_leg == 0:
                    # previous was right
                    leg = 'left'
                    self.goal_leg = 1
                    self.goal_x = 0
                    self.goal_y = 0
                    print leg
                    self.move_up(leg)

                    # get the goal
                    state = (self.rfoot_pos_com[0], -self.rfoot_pos_com[
                        1], self.rfoot_vel_com[0], -self.rfoot_vel_com[1])

                    x, y = self.get_goal(state)

                    self.current_state = state
                    self.current_cmd = (x, y)

                    self.move_down(leg, x, - y)

    def get_goal(self, state):

        xpos = self.Xpos.Discretize(state[0])
        ypos = self.Ypos.Discretize(state[1])
        xvel = self.Xvel.Discretize(state[2])
        yvel = self.Yvel.Discretize(state[3])

        # update the quasimetric
        if self.current_cmd is not None and not self.fall:
            oldxpos = self.Xpos.Discretize(self.current_state[0])
            oldypos = self.Ypos.Discretize(self.current_state[1])
            oldxvel = self.Xvel.Discretize(self.current_state[2])
            oldyvel = self.Yvel.Discretize(self.current_state[3])

            ux = self.Ux.Discretize(self.current_cmd[0])
            uy = self.Uy.Discretize(self.current_cmd[1])

            print 'UPDATE'
            self.quasix.OnlineUpdate(
                (oldxpos, oldxvel), (xpos, xvel), (self.current_cmd[0], ), 1.0)
            self.quasiy.OnlineUpdate(
                (oldypos, oldyvel), (ypos, yvel), (self.current_cmd[1], ), 1.0)

            s = '%d %d %d %d %d %d %d %d %d %d\n' % (
                oldxpos, oldypos, oldxvel, oldyvel, ux, uy, xpos, ypos, xvel, yvel)

            self.transitions_logd.write(s)
            self.transitions_logd.flush()

            s = '%f %f %f %f %f %f %f %f %f %f\n' % (self.current_state[0], self.current_state[1], self.current_state[
                2], self.current_state[3], self.current_cmd[0],
                self.current_cmd[1], state[0], state[1], state[2], state[3])

            self.transitions_log.write(s)
            self.transitions_log.flush()

        if self.fall:
            self.fall = False

        statex_d = (xpos, xvel)
        statey_d = (ypos, yvel)

        statex_idx = self.X.GetFlatIdx(statex_d)
        statey_idx = self.Y.GetFlatIdx(statey_d)

        uix = self.quasix.GetDrawnPolicy((xpos, xvel))
        utx = self.Ux.Continuize(uix)

        uiy = self.quasiy.GetDrawnPolicy((ypos, yvel))
        uty = self.Uy.Continuize(uiy)

        print 'GOAL', state, statex_d, statey_d, 'U', uix, uiy, utx, uty

        # utx = np.random.rand() * (self.Ux.maxs[0] - self.Ux.mins[0]) + \
        #     self.Ux.mins[0]
        # uty = np.random.rand() * (self.Uy.maxs[0] - self.Uy.mins[0]) + \
        #     self.Uy.mins[0]

        # print 'RAND:', utx, uty

        return (utx, uty)

    # lift the leg and get back to 0
    def move_up(self, leg):

        if leg == 'left':
            self.update()
            self.robot.lleg3d_up.add(
                0, 0, self.duration / 2.0)
            # self.robot.lleg3d_up.add(0, 0, 0.1)
            self.robot.lleg3d_up.start()
            self.robot.lleg3d_up.wait_to_start()
            self.robot.lleg3d_up.wait_to_stop()
            self.update()

        elif leg == 'right':

            self.update()
            self.robot.rleg3d_up.add(0, 0, self.duration / 2.0)

            # self.robot.rleg3d_up.add(0, 0, 0.1)
            self.robot.rleg3d_up.start()
            self.robot.rleg3d_up.wait_to_start()
            self.robot.rleg3d_up.wait_to_stop()

            self.update()

    # put foot on the ground at a specific pos
    def move_down(self, leg, x, y):

        if leg == 'left':

            self.robot.lleg3d_down.add(
                x, y, self.duration / 2.0)

            self.robot.lleg3d_down.start()
            self.robot.lleg3d_down.wait_to_start()
            self.robot.lleg3d_down.wait_to_stop()

        elif leg == 'right':

            self.robot.rleg3d_down.add(
                x, y, self.duration / 2.0)
            # self.robot.rleg3d_down.add( - 0.04 ,- 0.05 , self.duration / 2.0)
            self.robot.rleg3d_down.start()
            self.robot.rleg3d_down.wait_to_start()
            self.robot.rleg3d_down.wait_to_stop()


if __name__ == '__main__':

    # Define the problem variables

    Xpos = Distrib('Xpos', [XCARD], [XMIN], [XMAX], [False])
    Xvel = Distrib('Xvel', [DXCARD], [DXMIN], [DXMAX], [False])

    Ypos = Distrib('Ypos', [YCARD], [YMIN], [YMAX], [False])
    Yvel = Distrib('Yvel', [DYCARD], [DYMIN], [DYMAX], [False])

    Ux = Distrib('Ux', [UXCARD], [UXMIN], [UXMAX], [False])
    Uy = Distrib('Uy', [UYCARD], [UYMIN], [UYMAX], [False])

    X = Distrib(
        'XposXvel', [XCARD, DXCARD], [XMIN, DXMIN], [XMAX, DXMAX], [False, False])

    Y = Distrib(
        'YposYvel', [YCARD, DYCARD], [YMIN, DYMIN], [YMAX, DYMAX], [False, False])

    XY = Distrib(
        'XposYposXvelYvel', [XCARD, YCARD, DXCARD, DYCARD], [XMIN, YMIN, DXMIN, DYMIN], [XMAX, YMAX, DXMAX, DYMAX], [False, False, False, False])

    U = Distrib(
        'U', [UXCARD, UYCARD], [UXMIN, UYMIN], [UXMAX, UYMAX], [False, False])

    PosVel_PosVelU = CondDistrib(
        'Xpos Ypos Xvel Yvel', 'Xpos Ypos Xvel Yvel Ux Uy', [
            XCARD, YCARD, DXCARD, DYCARD],
        [XCARD, YCARD, DXCARD, DYCARD, UXCARD, UYCARD])

    XPosXVel_XPosXVelUx = CondDistrib(
        'Xpos Xvel', 'Xpos Xvel Ux', [XCARD, DXCARD],
        [XCARD, DXCARD, UXCARD])

    YPosYVel_YPosYVelUy = CondDistrib(
        'Ypos Yvel', 'Ypos Yvel Uy', [YCARD, DYCARD],
        [YCARD, DYCARD, UYCARD])

    Trx = Transition(X, Ux, XPosXVel_XPosXVelUx, None)
    Try = Transition(Y, Uy, YPosYVel_YPosYVelUy, None)

    print "Building graph"

    quasix = QM(Trx, lambda x, u: 1.0, LAMBDA=0.1)
    quasiy = QM(Try, lambda x, u: 1.0, LAMBDA=0.1)

    # quasi.beta = 0.001 #more flat softmin=more exploration

    quasix.Init()
    quasiy.Init()

    online_from_file_ind(
        sys.argv[1], quasix, quasiy, Xpos, Ypos, Xvel, Yvel, Ux, Uy)

    xpos = Xpos.Discretize(0.0)
    ypos = Ypos.Discretize(0.05)
    xvel = Xvel.Discretize(0.0)
    yvel = Yvel.Discretize(0.0)

    goalx = (xpos, xvel)
    goaly = (ypos, yvel)

    goal_vertx = quasix.Vertices[totuple(goalx)]
    goal_verty = quasiy.Vertices[totuple(goaly)]

    distx = quasix.Compute(goal_vertx)
    disty = quasiy.Compute(goal_verty)

    policyx = quasix.ComputePolicy()

    policyy = quasiy.ComputePolicy()

    print distx.reshape((XCARD, DXCARD))
    print
    print policyx.reshape((XCARD, DXCARD))

    print
    print disty.reshape((YCARD, DYCARD))
    print
    print policyy.reshape((YCARD, DYCARD))

    # stop

    # poppy = from_vrep(poppy_config, '127.0.0.1', 19997, 'poppy-standing-hightorque.ttt',
    # tracked_collisions=['CollisionRFoot', 'CollisionLFoot'])
    XP1 = Stab('poppy-standing-hightorque.ttt',
               tracked_collisions=['CollisionRFoot', 'CollisionLFoot'], gui=True)

    XP1.start(quasix=quasix, quasiy=quasiy, X=X, Y=Y, Xpos=Xpos,
              Ypos=Ypos, Xvel=Xvel, Yvel=Yvel, Ux=Ux, Uy=Uy)

    # XP2 = Stab('poppy-standing-hightorque.ttt',
    #            tracked_collisions=['CollisionRFoot', 'CollisionLFoot'],
    #            gui=False)

    # XP2.start(quasix=quasix, quasiy=quasiy, X=X, Y=Y, Xpos=Xpos,
    #           Ypos=Ypos, Xvel=Xvel, Yvel=Yvel, Ux=Ux, Uy=Uy)

    XP1.wait()

    # XP2.wait()
