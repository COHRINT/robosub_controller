from __future__ import division
import numpy as np
import control
import rospy

# A should ba 12x12
# B should be a 12x8
# C should be a 9x12 (12x12 with DVL)
# D should be a 9x8 (12x8 with DVL)

class Controller(object):
    def __init__(self):
        self.load_config()

    def load_config(self):
        '''Loads config file with all necessary values
        found in simulation'''
        pass

    def ROS_estimate(self):
        '''retrieves state estimate from UKF through
        ROS'''
        pass

    def linearize_matricies(self,x,u,dt):
        '''creates discritized linearized F,G,H&M matricies
        Input: x-linearization point
        u-linearization inputs
        dt-discrete interval'''
        pass

    def LQR(self):
        '''computes the gain matrix K to a specified behavior'''
        pass

    def feed_forward(self,A,B,C,K):
        '''creates the feed forward matrix F to compute
        control inputs using a desired state'''
        pass

    def sub_planner(self, points):
        '''given a set of points to hit on a trajectory,
        determine the desired state w/velocity to send
        to the controller'''
        pass

    def control_output(self,x,x_desired,type='moving'):
        '''uses a previously computed gain K to 
        create a control input for the motors
        x-current state
        x_desired-desired state
        type-which controller to use'''
        pass
    
    def test_controller(self):
        '''tests such as saturation and simulated responses'''
        pass

if __name__ == '__main__':
    pass
