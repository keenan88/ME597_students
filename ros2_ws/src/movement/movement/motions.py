# Imports
import rclpy

from rclpy.node import Node

#from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type = 0):
        
        super().__init__("motion_types")
        
        self.type = motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos_imu = QoSProfile(
            depth=10,                   # '_depth'
            history=QoSHistoryPolicy.KEEP_LAST,  # '_history'
            reliability=QoSReliabilityPolicy.RELIABLE,  # '_reliability'
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # '_durability'
#            lifespan=10,                # '_lifespan' in seconds
#            deadline=5,                 # '_deadline' in seconds
            liveliness=QoSLivelinessPolicy.AUTOMATIC,  # '_liveliness'
#            liveliness_lease_duration=5  # '_liveliness_lease_duration' in seconds
        )

        qos_encoder = QoSProfile(
            depth=10,                   # '_depth'
            history=QoSHistoryPolicy.KEEP_LAST,  # '_history'
            reliability=QoSReliabilityPolicy.RELIABLE,  # '_reliability'
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # '_durability'
#            lifespan=10,                # '_lifespan' in seconds
#            deadline=5,                 # '_deadline' in seconds
            liveliness=QoSLivelinessPolicy.AUTOMATIC,  # '_liveliness'
#            liveliness_lease_duration=5  # '_liveliness_lease_duration' in seconds
        )

        qos_scan = QoSProfile(
            depth=10,                   # '_depth'
            history=QoSHistoryPolicy.UNKNOWN,  # '_history'
            reliability=QoSReliabilityPolicy.RELIABLE,  # '_reliability'
            durability=QoSDurabilityPolicy.VOLATILE,  # '_durability'
#            lifespan=10,                # '_lifespan' in seconds
#            deadline=5,                 # '_deadline' in seconds
            liveliness=QoSLivelinessPolicy.AUTOMATIC,  # '_liveliness'
#            liveliness_lease_duration=5  # '_liveliness_lease_duration' in seconds
        )

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.encoder_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laserscan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback,  10)

        self.odom_initialized = True
        self.imu_initialized = True
        self.laser_initialized = True

        self.start_ns = self.get_clock().now().nanoseconds
        print("Star ns: ", self.start_ns)
        
        self.create_timer(0.1, self.timer_callback)
        


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        ...    # log imu msgs
        
    def odom_callback(self, odom_msg: Odometry):
        
        ... # log odom msgs
                
    def laser_callback(self, laser_msg: LaserScan):
        
        ... # log laser msgs with position msg at that time
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg = Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        
        msg=Twist()
        msg.angular.z = 6.28/8
        return msg

    def make_spiral_twist(self):
        msg=Twist()

        w = 6.28 / 16
        msg.angular.z = w

        r_inc = 0.1
        curr_ns = self.get_clock().now().nanoseconds
        print("curr ns:", curr_ns)
        print("ns diff:", curr_ns - self.start_ns)
        Vrx = w * r_inc * (curr_ns - self.start_ns) / (2 * 3.14 * 10e9) * 10

        msg.linear.x = Vrx
        ... # fill up the twist msg for spiral motion
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        ... # fill up the twist msg for line motion
        return msg

import argparse

def main(args=None):
    
    argParser=argparse.ArgumentParser(description="input the motion type")

    argParser.add_argument("--motion", type=str, default="spiral")

    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")

    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")


# Utilities.py starts here

from math import atan2, asin, sqrt

M_PI = 3.1415926535

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

            # TODO Part 5: Write the values from the list to the file
            ...
            
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
            # Skip the header line

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


# TODO Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    ... # just unpack yaw
    return yaw


