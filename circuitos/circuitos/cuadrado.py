import time

from sympy import true
import rclpy
import math
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion
from math import degrees, sqrt
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry




class avanzaPublisher(Node):

    def __init__(self):
        global x, y, th, x_i, y_i,th0
        x, y, th,th0, x_i, y_i = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        super().__init__('caminar')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom',
                                                 self.listener_callback, 10)
        self.flag = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.isX = True

    def timer_callback(self):
        global  x, y, x_i, y_i, th, th0
        move_cmd = Twist()

        if self.flag == 0:
            x_i, y_i = x, y
            th0 = th
            self.flag = 1
        elif self.flag == 1:
            print("Hola")
            d = math.sqrt( pow(x-x_i,2) + pow(y-y_i,2))
            if d < 0.80:
                move_cmd.angular.z = 0.0
                move_cmd.linear.x = 0.1

            else:
                move_cmd.linear.x = 0.0
                self.flag = 2
            self.pub.publish(move_cmd)
        elif self.flag == 2:
            if abs(th - th0) < 90.0:
                move_cmd.angular.z = 0.35
            else:
                self.isX = not self.isX
                move_cmd.angular.z = 0.0
                self.flag = 0
            move_cmd.linear.x = 0.0
            self.pub.publish(move_cmd)

    def listener_callback(self, data):
        global x, y, th
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        q4 = data.pose.pose.orientation.w
        q = (q1, q2, q3, q4)
        e = euler_from_quaternion(q)
        th = degrees(e[2])
        print(f"(x:{x}, y:{y}, th:{th})")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = avanzaPublisher()
    rclpy.spin(minimal_publisher)
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
