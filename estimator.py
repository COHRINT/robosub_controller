from __future__ import division
import numpy as np
from scipy import integrate
import rospy

class UKF(object):
    def __init__(self):
        self.load_config()
        self.make_constants(constants)

    def make_constants(self,constants):
        '''function designed to compute common strings
        of constants used in all matricies to speed up
        computation
        Input: constants-constants from modeling/config
        file'''

        pass

    def ROS_sensors(self):
        '''get sensor measurements from ROS'''
        pass
    
    def propagate_pred(self,point):
        '''propagates a single sigma point according to
        the non-linear dynamics of the system for the 
        prediction step'''
        pass

    def prediction(self,points):
        '''given a set of sigma points, combine to produce
        a predicted mean and covariance'''
        pass

    def propagate_meas(self,point):
        '''propagates a single sigma point according to
        the non-linear dynamics of the system for the 
        measurement step'''
        pass

    def state_update(self):
        '''calculate kalman gain and perform final
        state estimate update for the non-linear dynamics'''
        pass

if __name__ == '__main__':
    pass
