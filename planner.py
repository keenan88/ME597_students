# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

import numpy as np
from math import atan2

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
        x = np.linspace(0, 1, 10)
        y = (x + 1) * (x + 1)
        y = -x * x

        trajectory_points = []

        for i in range(len(x)):


            theta = round(atan2(y[i], x[i]), 1)

            trajectory_points.append([round(x[i], 2), round(y[i], 2), theta])

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        return trajectory_points

