#! /usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
# from gazebo_msgs.msg import LinkStates
# from geometry_msgs.msg import Pose, Twist
import numpy as np
# import math


class lidar_controller:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        self.straight_speed = 50.0
        self.rotation_speed = 50.0
        self.left_speed = 0.0
        self.right_speed = 0.0

        self.left_wheel_pub = rospy.Publisher("/robot/left_wheel_controller/command", Float64, queue_size=10)
        self.right_wheel_pub = rospy.Publisher("/robot/right_wheel_controller/command", Float64, queue_size=10)

        self.lidar_sub = rospy.Subscriber("/robot/laser/scan", LaserScan, self.lidar_cb)
        #self.link_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_cb)

    # def link_states_cb(self, data):
    #     self.left_wheel_pub.publish(5.0)
    #     quant_base_link = data.pose[1].orientation
    #     yaw_ang = math.acos(quant_base_link.w)*2
    #     #print(yaw_ang*180/math.pi)
    #     print(math.atan2(quant_base_link.z, quant_base_link.w)*360/math.pi)


    def lidar_cb(self, data):
        ang = np.arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        collision_with_front = np.array(data.ranges) * np.sin(ang)
        if any(abs(collision_with_front) < 0.2):
            print("Avoiding obstacle.")
            if np.argmin(collision_with_front) < collision_with_front.size/2:
                self.left_speed = self.straight_speed + self.rotation_speed
                self.right_speed = self.straight_speed - self.rotation_speed
            else:
                self.left_speed = self.straight_speed - self.rotation_speed
                self.right_speed = self.straight_speed + self.rotation_speed   
        else:
            print('Moving forward.')
            self.left_speed = self.straight_speed
            self.right_speed = self.straight_speed

        self.left_wheel_pub.publish(self.left_speed)
        self.right_wheel_pub.publish(self.right_speed)


if __name__ == '__main__':
    lidar_controller("controller")
    rospy.spin()
