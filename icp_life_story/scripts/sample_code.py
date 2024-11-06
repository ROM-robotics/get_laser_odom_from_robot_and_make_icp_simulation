import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import os

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Locate the package path for saving the file
        package_path = get_package_share_directory('your_package_name')
        output_file_path = os.path.join(package_path, 'laser_odom_data.dat')
        
        # Open the file for writing
        self.file = open(output_file_path, "w")
        
        # Subscribers
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Initialize data variables
        self.index = 0
        self.latest_odom = None
    
    def laser_callback(self, msg):
        # Increment index for each laser scan message received
        self.index += 1
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')  # Current time with microseconds
        num_scans = len(msg.ranges)
        
        # Format scan data and limit to first 5 values for readability (or adjust as needed)
        scan_values = ', '.join(f"{val:.2f}" for val in msg.ranges[:5]) + (" ..." if num_scans > 5 else "")
        
        # Retrieve odometry data if available
        if self.latest_odom:
            x = self.latest_odom.pose.pose.position.x
            y = self.latest_odom.pose.pose.position.y
            odometry_data = f"({x:.2f}, {y:.2f}, 0)"
        else:
            odometry_data = "(None, None, 0)"  # Default if no odometry data yet
        
        # Data acquisition time
        acquisition_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Write formatted data to file
        data_entry = (
            f'LASERSCAN {self.index} {current_time} {num_scans} {scan_values} '
            f'{odometry_data} {acquisition_time}\n'
        )
        self.file.write(data_entry)
        self.file.flush()  # Ensure immediate write to disk

    def odom_callback(self, msg):
        # Update latest odometry data
        self.latest_odom = msg

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
