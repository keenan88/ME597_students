# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

import numpy as np
from math import atan2, exp

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        x = np.linspace(0, 2, 25)
        

        trajectory_points = []

        for i in range(len(x)):

            #y = -x[i]*x[i]

            y = 1 / (1 + exp(1.5 * x[i])) - 0.5

            theta = round(atan2(y, x[i]), 1)

            trajectory_points.append([round(x[i], 2), round(y, 2), theta])

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return trajectory_points

