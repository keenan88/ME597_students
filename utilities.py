from math import atan2, asin, sqrt
from transformations import euler_from_quaternion as efq

from math import sqrt, pi

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """

    quat_as_array = [
        quat.x,
        quat.y,
        quat.z,
        quat.w
    ]
    roll, pitch, yaw = efq(quat_as_array)

    #print(roll, pitch, yaw)

    # just unpack yaw
    return roll # Unexpected that roll would be yaw, might be some frame issue


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
    
    x_err = current_pose[0] - goal_pose[0]
    y_err = current_pose[1] - goal_pose[1]
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y,theta]
    # Remember to use the Euclidean distance to calculate the error.
    error_linear= sqrt(x_err * x_err + y_err * y_err)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y,theta]
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal


    error_angular = goal_pose[2] - current_pose[2]


    # Remember to handle the cases where the angular error might exceed the range [-π, π]
    #error_angular %= pi
         
    return error_angular
