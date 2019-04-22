#!/usr/bin/env python

"""
Sensor nodes, used to simulated sensor measurements from ground truth data.
"""

import os
import sys
import yaml
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from robosub_controller.msg import SimState

class Sensor(object):
    """
    Generic sensor class. Meant to be subclassed.
    """
    def __init__(self,cfg_path):

        self.cfg = self.load_config(cfg_path)

        # get update rate
        self.update_rate = self.cfg['update_rate']

        # get node name
        self.node_name = self.cfg['type']


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

class ImuSensor(Sensor):
    """
    Simulated IMU, outputs accelerations and angular rates. Assumes zero-mean gaussian noise.
    """

    def __init__(self,cfg_path):
        super(ImuSensor,self).__init__(cfg_path)

        self.accel_noise = np.array(self.cfg['accel_noise'])
        self.gyro_noise = np.array(self.cfg['gyro_noise'])

        # init rosnode
        rospy.init_node(self.node_name)

        # subcribe to ground truth topic
        rospy.Subscriber('/ground_truth',SimState,self.ground_truth_cb)

        # create publisher
        self.pub = rospy.Publisher('/imu',Imu,queue_size=10)

        # wait for messages
        rospy.spin()

    def ground_truth_cb(self,msg):
        """
        Callback function to get ground truth imu messages and add noise then rebpublish.
        """
        # create accel noise and gyro noise
        accel_noise = np.random.multivariate_normal([0,0,0],self.accel_noise)
        gyro_noise = np.random.multivariate_normal([0,0,0],self.gyro_noise)
        
        # create outgoing message
        out_msg = Imu()

        # add noise to acceleration
        out_msg.linear_acceleration.x = msg.acceleration.linear.x + accel_noise[0]
        out_msg.linear_acceleration.y = msg.acceleration.linear.y + accel_noise[1]
        out_msg.linear_acceleration.z = msg.acceleration.linear.z + accel_noise[2]

        # add noise to gyro
        out_msg.angular_velocity.x = msg.velocity.angular.x + gyro_noise[0]
        out_msg.angular_velocity.y = msg.velocity.angular.y + gyro_noise[1]
        out_msg.angular_velocity.z = msg.velocity.angular.z + gyro_noise[2]
        
        self.pub.publish(out_msg)

class DepthSensor(Sensor):
    """
    Simulated depth sensor. Assumes zero-mean gaussian noise.
    """

    def __init__(self,cfg_path):
        super(DepthSensor,self).__init__(cfg_path)

        self.depth_noise = np.array(self.cfg['depth_noise'])

        # init rosnode
        rospy.init_node(self.node_name)

        # subscribe to sub position
        rospy.Subscriber('/ground_truth',SimState,self.ground_truth_cb)

        # create publisher
        self.pub = rospy.Publisher('/depth',Float64,queue_size=10)

        # wait for messages
        rospy.spin()

    def ground_truth_cb(self,msg):
        """
        Callback for ground truth data from simulator.
        """
        # create depth noise
        depth_noise = np.random.normal(0,self.depth_noise)

        # create and publish msg
        depth = msg.pose.position.z + depth_noise
        out_msg = Float64(data=depth)
        self.pub.publish(out_msg)


if __name__ == "__main__":

    # get sensor type to be started
    sensor_type = sys.argv[1]

    cfg_path = '../config/{}_config.yaml'.format(sensor_type)

    if sensor_type == 'imu':
        ImuSensor(cfg_path)
    elif sensor_type == 'depth':
        DepthSensor(cfg_path)