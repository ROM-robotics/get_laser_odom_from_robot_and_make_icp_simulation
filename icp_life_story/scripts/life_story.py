#!/usr/bin/python3

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion



class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('life_story_node')
        
    
        pkg = get_package_share_directory('icp_life_story')
        # save_file = os.path.join(pkg, 'laser_odom_data.dat')
        
        
        # self.file = open(save_file, "w")
        
        # output_file_path = '/ros2_ws/src/developer_packages/icp_life_story/test.dat'
        # Open the file for writing
        self.file = open("test.dat", "w")
        
        self.laser_subscription = self.create_subscription(LaserScan,'/scan',self.laser_callback,10)   # SUBSCRIBER
        self.odom_subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        self.cur_time =  rclpy.clock.Clock().now()
        self.cur_time_sec = self.cur_time.seconds_nanoseconds()[0]
        self.cur_time_nanosec = self.cur_time.seconds_nanoseconds()[1]

        # scan params
        scan_no_max = 726 #for simpleURG
        self.scan = [0]*(scan_no_max*2) #① Which measurement angle+② Double the number of elements to save in the scan data set
        self.scan_no = 0
        self.angle_min = -2.35619449615 #for simpleURG -135deg
        self.angle_max = 2.09234976768 #for simpleURG 120deg
        self.angle_increment = 0.00613592332229 #for simpleURG 0.36deg
        self.angle_total_n = scan_no_max
        self.scan_time_secs = 0
        self.scan_time_nsecs = 0  

        #odom params

        # self.odom_time_secs 
        # self.odom_time_nsecs

        self.position_x = 0.0           # မလိုတာတွေဖျက်ရန်
        self.position_y = 0.0
        self.position_z = 0.0

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        self.yaw = 0.0
        

    def laser_callback(self, message):
        # self.scan_no = message.header.seq      # ros2 မှာ seq မရှိ

        self.scan_no += 1

        self.scan_time_secs = message.header.stamp.sec
        self.scan_time_nsecs = message.header.stamp.nanosec
        for i in range(0, self.angle_total_n, 2):
            # Calculate the measurement angle in degrees
            angle_deg = (self.angle_min + i * self.angle_increment) * 180 / 3.14
            self.scan[int(i / 2)] = angle_deg

            # Check if the range data at index i is NaN and handle it
            if i < len(message.ranges) and math.isnan(message.ranges[i]):
                # Assign 0 if the data is NaN
                self.scan[int(i) + 1] = 0
            elif i < len(message.ranges):
                # Otherwise, use the actual data
                self.scan[int(i) + 1] = message.ranges[i]
            else:
                # Handle any out-of-range indices (optional)
                self.scan[int(i) + 1] = 0
    
    def odom_callback(self, message):

        # self.odom_time_secs = message.header.stamp.sec       # odom time stamp ယူရန်
        # self.odom_time_nsecs = message.header.stamp.nanosec

        self.position_x = message.pose.pose.position.x
        self.position_y = message.pose.pose.position.y
        self.position_z = message.pose.pose.position.z

        self.orientation_x = message.pose.pose.orientation.x
        self.orientation_y = message.pose.pose.orientation.y
        self.orientation_z = message.pose.pose.orientation.z
        self.orientation_w = message.pose.pose.orientation.w

        orientation_q = message.pose.pose.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
    )
        
        self.write_data()
    
    # Convert quaternion to Euler angles
        roll, pitch, self.yaw = euler_from_quaternion(quaternion) # yaw ထုတ်


        
        # position = message.pose.pose.position
        # orientation = message.pose.pose.orientation
        # odom_data = (
        #     f"Odometry: Position -> x: {position.x}, y: {position.y}, z: {position.z} "
        #     f"Orientation -> x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}\n"
        # )
        # self.file.write(odom_data)
        # self.file.flush()  # Ensures data is written immediately
    
    def write_data(self):

        if self.scan_no > 0 : #Make the same format as the input data file of SLAM in this book
            output_list =["LASERSCAN"]
            output_list.append(self.scan_no)
            output_list.append(self.cur_time_sec)     #မူရင်းမှာ scan time မယူ (node time ပဲယူ)
            output_list.append(self.cur_time_nanosec)
            output_list.append('340') #(723-44)/2 Save scan data from the sensor in half.
            for i in range (44, 724, 1): #Roughly saves scan data from -120deg to 120deg
                output_list.append(self.scan[i])
            output_list.append(self.position_x) #Odometry X direction
            output_list.append(self.position_y) #Odometry Y direction
            output_list.append(self.yaw) #Save with Odometry rotation angle RAD
            output_list.append(self.scan_time_secs)
            output_list.append(self.scan_time_nsecs)  # မူရင်းမှာ odom time stamp မယူထား (scan_time ယူ)

            # output_list.append(1)
            # output_list.append(2.97)     #မူရင်းမှာ scan time မယူ (node time ပဲယူ)
            # output_list.append(432)
            # output_list.append('340') #(723-44)/2 Save scan data from the sensor in half.
            # for i in range (44, 724, 1): #Roughly saves scan data from -120deg to 120deg
            #     output_list.append(4)
            # output_list.append(self.position_x) #Odometry X direction
            # output_list.append(self.position_y) #Odometry Y direction
            # output_list.append(self.yaw) #Save with Odometry rotation angle RAD
            # output_list.append(self.scan_time_secs)
            # output_list.append(self.scan_time_nsecs)  # မူရင်းမှာ odom time stamp မယူထား (scan_time ယူ)
            for d in output_list:
                self.file.write("%s " %d)
            self.file.write("\n")


    def destroy_node(self):
        # Ensure the file is closed when the node is destroyed
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    data_logger_node = DataLoggerNode()
    
    try:
        rclpy.spin(data_logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        data_logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
