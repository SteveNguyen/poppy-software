#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: 'test_simple_biped.py'
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen@inria.fr
#  Created	: vendredi, janvier 16 2015
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

from pyvrep.xp import VrepXp

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
import os
import poppytools
import json


class SimpleBipedVrepXp(VrepXp):

    def __init__(self, scene, tracked_collisions=[], process=False, gui=False):
        configfile = os.path.join(os.path.dirname(poppytools.__file__),
                                  'configuration', 'simple_biped_config.json')

        with open(configfile) as f:
            simple_biped_config = json.load(f)

        VrepXp.__init__(
            self, simple_biped_config, scene, tracked_collisions, process, gui)


class Biped(SimpleBipedVrepXp):

    def Init(self):

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

        # self.lfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
        #     'foot_left_visual'], self.dummies_h['CoM'], streaming=True)

        # self.rfoot = self.vrepio.call_remote_api('simxGetObjectPosition', self.obj_h[
        #     'foot_right_visual'], self.dummies_h['CoM'], streaming=True)

        # self.com_abs = self.vrepio.call_remote_api(
        #     'simxGetObjectPosition', self.dummies_h['CoM'], - 1, streaming=True)

        # self.com_vel = self.vrepio.call_remote_api(
        #     'simxGetObjectVelocity', self.dummies_h['CoM'], streaming=True)

        print self.obj_h
        print
        print self.joint_h
        print
        print self.dummies_h
        self.state = 0
        # print self.vrepio.call_remote_api('simxGetJointPosition',
        # self.joint_h['l_hip_roll'])

        self.vrepio.call_remote_api(
            'simxSetJointTargetPosition', self.joint_h['l_hip_roll'], 10)
        print self.vrepio.call_remote_api('simxGetJointPosition', self.joint_h['l_hip_roll'])

    def run(self):
        self.Init()
        print 'Init'
        # self.robot.reset_simulation()
        self.vrepio.restart_simulation()
        while True:
            # for m in self.robot.motors:
            #     print m.name, m.present_position
            #     m.goal_position += 0.01

            # print self.vrepio.call_remote_api('simxGetJointPosition',
                                              # self.joint_h['l_hip_roll'], 0)

            # self.robot.l_hip_roll.goal_position = 10.0
            # print self.robot.l_hip_roll.present_position
            # time.sleep(0.1)
            # print self.state
            if self.state == 0 or self.state == 1:
                # self.robot.l_leg_h.goal_position = 0.1
                self.vrepio.call_remote_api(
                    'simxSetJointTargetPosition', self.joint_h['l_leg_h'], 0.1)
                systime.sleep(0.1)
                # self.robot.l_leg_h.goal_position = 0.0
                self.vrepio.call_remote_api(
                    'simxSetJointTargetPosition', self.joint_h['l_leg_h'], 0.0)
                self.state = 2

            elif self.state == 2:
                # self.robot.r_leg_h.goal_position = 0.1
                self.vrepio.call_remote_api(
                    'simxSetJointTargetPosition', self.joint_h['r_leg_h'], 0.1)
                systime.sleep(0.1)
                # self.robot.r_leg_h.goal_position = 0.0
                self.vrepio.call_remote_api(
                    'simxSetJointTargetPosition', self.joint_h['r_leg_h'], 0.0)
                self.state = 1


if __name__ == '__main__':

    XP1 = Biped('simple_biped.ttt', gui=True)

    XP1.start()

    XP1.wait()
