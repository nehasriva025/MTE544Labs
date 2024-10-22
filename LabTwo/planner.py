# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

PARABOLA_TRAJECTORY = 0
SIGMOID_TRAJECTORY = 1

import numpy as np



class planner:
    def __init__(self, type_):

        self.type=type_

        #Set trajectory here 
        self.trajectory_type = PARABOLA_TRAJECTORY

    
    def plan(self, goalPoint=[-1.0, -1.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        points_list = []
        range = 0
        if self.trajectory_type == PARABOLA_TRAJECTORY: 
            range = np.linspace(0, 1.5, 10)
            for x in range: 
                y = x **2 
                points_list.append([x,y])
        elif self.trajectory_type == SIGMOID_TRAJECTORY: 
            range = np.linspace(0, 2.5, 10)
            for x in range: 
                y = 2 /(1 + np.exp(-2 * x)) -1 
                points_list.append([x,y])

        return points_list

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

