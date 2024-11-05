#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
# from raspimouse_ros_2.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger
# from raspimouse_ros_2.srv import TimedMotion
from nav_msgs.msg import Odometry

from icp_motor_control.msg import MotorFreqs
from icp_motor_control.srv import TimedMotion
import tf_transformations
from tf2_ros import TransformBroadcaster

class Motor(Node):
    def __init__(self):
        super().__init__('motors')

        # Initialize motor power and set shutdown hook
        if not self.set_power(False):
            self.get_logger().error("Cannot initialize motor power")
            self.destroy_node()
            return

        self.sub_raw = self.create_subscription(MotorFreqs, 'motor_raw', self.callback_raw_freq, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, 10)
        self.srv_on = self.create_service(Trigger, 'motor_on', self.callback_on)
        self.srv_off = self.create_service(Trigger, 'motor_off', self.callback_off)
        self.srv_tm = self.create_service(TimedMotion, 'timed_motion', self.callback_tm)

        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.bc_odom = TransformBroadcaster(self)

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
        self.cur_time = self.get_clock().now()
        self.last_time = self.cur_time

        self.using_cmd_vel = False
        self.create_timer(0.1, self.send_odom)

    def set_power(self, onoff=False):  # for test set it true (gg)
        en = "/dev/rtmotoren0"
        try:
            with open(en, 'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            self.get_logger().error("Cannot write to " + en)
            return False

    def set_raw_freq(self, left_hz, right_hz):
        if not self.is_on:
            self.get_logger().error("Motor is not powered on")
            return

        try:
            with open("/dev/rtmotor_raw_l0", 'w') as lf, open("/dev/rtmotor_raw_r0", 'w') as rf:
                lf.write(f"{int(round(left_hz))}\n")
                rf.write(f"{int(round(right_hz))}\n")
        except:
            self.get_logger().error("Cannot write to motor raw files")

    def callback_raw_freq(self, message):
        self.set_raw_freq(message.left_hz, message.right_hz)

    def callback_cmd_vel(self, message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        self.vth = message.angular.z

        forward_hz = 80000.0 * message.linear.x / (9 * math.pi)
        rot_hz = 400.0 * message.angular.z / math.pi
        self.set_raw_freq(forward_hz - rot_hz, forward_hz + rot_hz)

        self.using_cmd_vel = True
        self.last_time = self.get_clock().now()

    # def onoff_response(self, onoff):  #  triggerResponse က ros2 မှာ မရှိ
    #     response = TriggerResponse()
    #     response.success = self.set_power(onoff)
    #     response.message = "ON" if self.is_on else "OFF"
    #     return response

    def callback_on(self, request, response):
        response.success = self.set_power(True)
        response.message = "ON" if self.is_on else "OFF"
        return response

    def callback_off(self, request, response):
        response.success = self.set_power(False)
        response.message = "ON" if self.is_on else "OFF"
        return response

    def callback_tm(self, request, response):
        if not self.is_on:
            self.get_logger().error("Motor is not powered on")
            response.success = False
            return response

        dev = "/dev/rtmotor0"
        try:
            with open(dev, 'w') as f:
                f.write(f"{request.left_hz} {request.right_hz} {request.duration_ms}\n")
            response.success = True
        except:
            self.get_logger().error("Cannot write to " + dev)
            response.success = False
        return response


        dev = "/dev/rtmotor0"
        try:
            with open(dev, 'w') as f:
                f.write(f"{request.left_hz} {request.right_hz} {request.duration_ms}\n")
            response.success = True
        except:
            self.get_logger().error("Cannot write to " + dev)
            response.success = False
        return response

    def send_odom(self):
        self.cur_time = self.get_clock().now()
        dt = (self.cur_time - self.last_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds

        # Update position and orientation
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        # Prepare quaternion for the transform
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        
        # Publish the transform over tf
        t = TransformStamped()
        t.header.stamp = self.cur_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.bc_odom.sendTransform(t)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.cur_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time


def main(args=None):
    rclpy.init(args=args)
    motor_node = Motor()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
