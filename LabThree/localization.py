import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
imu_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
#May need to do a define QoS profile based on the information available in "ros2 topic info /odom --verbose" "ros2 topic info /imu --verbose"


class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        #Using np.arrays for the matrices
        #Initial state as 0s 
        x= np.array([0,0,0,0,0,0])

        #Initialize values for the noise 
        #For testing - modify these values to see the effect of the noise
        Q_value = 0.5
        R_value = 0.5

        #Given inital value * Identity matrix = diagonal matrix of given value
        # Q is nxn where n = 6 
        Q = Q_value*np.eye(6)

        # R is bxb where b = 4
        R = R_value*np.eye(4)
        
        #Set initial P to Q (as stated in tutorial)
        P= Q_value*np.eye(6) # initial covariance
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom", qos_profile=odom_qos)
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu", qos_profile=imu_qos)
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        
        #Getting the sensor data
        odom_velx = odom_msg.twist.twist.linear.x
        odom_vely = odom_msg.twist.twist.linear.y
        odom_omega = odom_msg.twist.twist.angular.z
        odom_v = np.sqrt(odom_velx**2 + odom_vely**2)
        odom_timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        imu_ax = imu_msg.linear_acceleration.x
        imu_ay = imu_msg.linear_acceleration.y

        #Formatting into measurement vector
        z = np.array([odom_v, odom_omega, imu_ax, imu_ay]) 
        
        # Implement the two steps for estimation
        #Prediction step 
        self.kf.predict()

        #Update step
        self.kf.update(z)
        
        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        #From tutorial: vdot = kf_vdot = kf_ax
        #therefore, naming vdot -> ax
        kf_x, kf_y, kf_th, kf_w, kf_v, kf_ax = xhat

        self.pose=np.array(kf_x, kf_y, kf_th, odom_timestamp)

        # TODO Part 4: log your data
        #From tutorial: 
        kf_ay = kf_v * kf_w #Getting y component of accel 
        kf_vx = kf_v * np.cos(kf_th) #Getting x component of velocity 

        #Log the values stated from the tutorial 
        self.loc_logger.log_values([imu_ax, imu_ay, kf_ax, kf_ay, kf_vx, kf_w, kf_x, kf_y, odom_timestamp])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
