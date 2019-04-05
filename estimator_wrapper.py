#!/usr/bin/env python

"""
ROS wrapper for RoboSub control system estimator. Queue measurement messages
from sensors and runs an instance of a UKF.
"""

import rospy

from .estimator import UKF
from .helpers import load_config

class EstimatorWrapper(object):
    """
    ROS wrapper for RoboSub control system estimator. Queues measurement messages
    from sensors and runs an instance of a UKF.
    """  

    def __init__(self,filter_cfg,ros_cfg):
        """
        Inputs:

            filter_cfg  -- dictionary of parameters for filter
            ros_cfg     -- dictionary of parameters for ros node
        """
        
        # create filter
        self.filter = UKF(**filter_cfg)

        # initialize ros node
        rospy.init_node('estimator')

        # subscribe to sensors
        rospy.Subscriber(imu_topic, ImuMsg, self.imu_callback)
        rospy.Subscriber(depth_topic, DepthMsg, self.depth_callback)

        # create publisher for state estimate and covariance
        self.pub = rospy.Publisher(estimator_topic, StateMsg)

        # start filter update loop
        while not rospy.is_shutdown():

            # update filter
            self.filter.update()

            # generate state msg
            msg = self.gen_state_msg(self.filter.x)
            self.pub.publish(msg)

    def gen_state_msg(self,x,P=None):
        """
        Generates state estimate message to be published.
        """
        msg = StateMsg()
        # code here to populate message

        return msg


    def imu_callback(self,msg):
        """
        IMU measurement callback function.
        Add measurement to measurement queue.

        Inputs:

            msg -- IMU measurement ROS message

        Returns:

            none
        """
        self.filter.measurement_queue.put(msg)

    def depth_callback(self,msg):
        """
        Depth sensor measurment callback function.
        Adds measurement to measurement queue.

        Inputs:

            msg -- depth sensor measurement ROS message

        Returns:
        
            none
        """
        self.filter.measurement_queue.put(msg)

if __name__ == "__main__":
    
    # load filter and ros configs
    filter_cfg_fn = '../config/config.yaml'
    filter_cfg = load_config(filter_cfg_fn)

    ros_cfg_fn = '../config/ros_config.yaml'
    ros_cfg = load_config(cfg_fn)

    # instantiate wrapper
    EstimatorWrapper(filter_cfg=filter_cfg,**ros_cfg)