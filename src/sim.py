#!/usr/bin/env python

"""
Simple simulator for Robosub. Assumes point mass.
"""

import os
import sys
import yaml
import rospy
import tf
import numpy as np

from robosub_controller.msg import SimState

class Simulator(object):
    """
    Simple simulator for RoboSub minisub. Propagates vehicle dynamics with
    control input and publishes pose, twist, and accelerations.
    """
    def __init__(self,cfg_path):

        cfg = self.load_config(cfg_path)

        self.update_rate = cfg['update_rate']
        self.dyn_noise = cfg['noise']
        self.sim_time = 0
        self.dt = cfg['dt']

        self.control_input = np.array([0,0,0,0,0,0,0])

        self.sim_state = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

        # initialize node
        rospy.init_node('sim')

        # subscribe to controller input
        # rospy.Subscriber('/control_input',ControlInput,self.control_intput_cb)

        # create publisher
        self.pub = rospy.Publisher('/ground_truth',SimState,queue_size=10)

        # start running update loop
        loop_rate = rospy.Duration(self.update_rate)
        rospy.loginfo('Initialized simulation node. Starting loop at {} Hz'.format(str(self.update_rate)))

        while not rospy.is_shutdown():

            # update state of the world
            self.update()

            # publish updated world state
            msg = SimState()
            msg.pose.position.x = self.sim_state[0]
            msg.pose.position.y = self.sim_state[1]
            msg.pose.position.z = self.sim_state[2]

            orientation = tf.transformations.quaternion_from_euler(
                                    self.sim_state[4],
                                    self.sim_state[5],
                                    self.sim_state[6]
            )
            msg.pose.orientation.x = orientation[0]
            msg.pose.orientation.y = orientation[1]
            msg.pose.orientation.z = orientation[2]
            msg.pose.orientation.w = orientation[3]

            msg.velocity.linear.x = self.sim_state[6]
            msg.velocity.linear.y = self.sim_state[7]
            msg.velocity.linear.z = self.sim_state[8]

            msg.velocity.angular.x = self.sim_state[9]
            msg.velocity.angular.y = self.sim_state[10]
            msg.velocity.angular.z = self.sim_state[11]

            msg.acceleration.linear.x = self.sim_state[12]
            msg.acceleration.linear.y = self.sim_state[13]
            msg.acceleration.linear.z = self.sim_state[14]

            msg.acceleration.angular.x = self.sim_state[15]
            msg.acceleration.angular.y = self.sim_state[16]
            msg.acceleration.angular.z = self.sim_state[17]

            self.pub.publish(msg)

            self.sim_time += self.dt
            rospy.sleep(loop_rate)

    def load_config(self,cfg_path):
        """
        Load config file for sensor.

        Inputs:

            cfg_path -- relative path to config file

        Returns:

            cfg -- config dictionary
        """
        fn = os.path.abspath(os.path.join(os.path.dirname(__file__),cfg_path))
        try:
            with open(fn,'r') as f:
                cfg = yaml.load(f)
        except IOError as e:
            print('Could not load config file: {}'.format(e))

        return cfg

    def update(self):
        """
        Updates the sim world state according to sub dynamics.
        
        Inputs:

            none

        Returns:

            none
        """
        # call dynamics fxn to update
        # self.sim_state = dynamics(self.sim_state)
        self.sim_state += 1

    def control_input_cb(self,msg):
        """
        Control input callback. Saves most recent control input.
        """
        self.control_input = np.array(msg.data)

if __name__ == "__main__":

    save_path = '../config/sim_config.yaml'

    Simulator(save_path)
