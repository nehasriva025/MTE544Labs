import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        #Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        #Defining QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3

        odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
 
            self.subscription = self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, pose_msg: odom):
        
        #Read x,y, theta, and record the stamp
        odom_timestamp = pose_msg.header.stamp # gets timestamp
        odom_quat = pose_msg.pose.pose.orientation # sets quat as orientation
        odom_th = euler_from_quaternion(odom_quat) # gets theta position
        odom_x = pose_msg.pose.pose.position.x # gets x position
        odom_y = pose_msg.pose.pose.position.y # gets y position
        self.pose=[odom_x, odom_y, odom_th, odom_timestamp]
        # Log the data
        self.loc_logger.log_values([self.pose[0], self.pose[1], self.pose[2], Time.from_msg(self.pose[3]).nanoseconds])
    
    def getPose(self):
        return self.pose


# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
    
if __name__=="__main__":
    
    init()
    # Create the node
    localizer = localization()
    # Spin the node
    spin(localizer)