# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...

P=0; PD=1; PI=2; PID=3


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #Create a publisher for the topic responsible for robot's motion
        self.publisher= self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        #Parameter tuning
        #Based on the type of motion, the controller is instantiated with different parameters
        if motion_type == POINT_PLANNER:
            # self.controller=controller(klp=0.4, kap=0.9, controller=P)
            # self.controller=controller(klp=0.4, klv=0.3, kap=0.9, kav=0.2, controller=PD)
            # self.controller=controller(klp=0.4, kli=0.6, kap=0.9, kai=0.6, controller=PI)
            # self.controller=controller(klp=0.4, klv=0.3, kli=0.6, kap=0.9, kav=0.2, kai=0.6, controller=PID)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            # self.controller=trajectoryController(klp=0.4, kap=0.9, controller=P)
            # self.controller=trajectoryController(klp=0.4, klv=0.9, kap=0.9, kav=0.9, controller=PD)
            # self.controller=trajectoryController(klp=0.4, kli=2.2, kap=0.9, kai=0.9, controller=PI)
            self.controller = trajectoryController(klp=0.5, klv=1.0, kli=0.1, kap=0.9, kav=0.9, kai=0.1, controller=PID)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)
        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        #Run the localization node
        spin_once(self.localizer)
        currPos = self.localizer.getPose()

        if self.localizer.getPose is None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Check if you reached the goal
        if type(self.goal) == list: # If its a trajectory 
            currLinErr = abs(calculate_linear_error(currPos,self.goal[-1]))
            currAngErr = abs(calculate_angular_error(currPos,self.goal[-1]))
            #Checking if the magnitude of the error is less than a certain threshold 
            reached_goal = currAngErr < 0.1 and currLinErr < 0.1
        else: #Else its point planner 
            currLinErr = abs(calculate_linear_error(currPos,self.goal))
            currAngErr = abs(calculate_angular_error(currPos,self.goal))
            #Checking if the magnitude of the error is less than a certain threshold 
            reached_goal = currAngErr < 0.1 and currLinErr < 0.01
    

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #Exit the spin
            raise SystemExit 

        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)
        
import argparse


def main(args=None):
    
    init()

    #You might need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Defining QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    

    #Instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist,'/cmd_vel',odom_qos,[-1,-1],10,POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist,'/cmd_vel',odom_qos,[1,1],10,TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
