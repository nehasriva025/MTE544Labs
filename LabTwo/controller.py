import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    #Added a controller type to the constructor to allow for P, PI, PD, and PID controllers
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, controller=0):
        
        #Testing PD, PI, and PID controller
        self.PID_linear=PID_ctrl(controller, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(controller, kap, kav, kai, filename_="angular.csv")


    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)


        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        #Adding saturation limits 

        #Max for robot (in lab)
        # linear_max = 0.31 
        # angular_max = 1.90 

        #Max for simulation 
        linear_max = 0.22 
        angular_max = 2.84 

        #change to 0.31 for the real robot
        linear_vel = linear_max if linear_vel > linear_max else linear_vel
        #change to 1.90  for the real robot
        angular_vel= angular_max if angular_vel > angular_max else angular_vel
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, controller=0):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        #Adding saturation limits

        #Max for robot (in lab)
        linear_max = 0.31 
        angular_max = 1.90 

        #Max for simulation 
        # linear_max = 0.22 
        # angular_max = 2.84 

        linear_vel = linear_max if linear_vel > linear_max else linear_vel
        angular_vel= angular_max if angular_vel > angular_max else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
