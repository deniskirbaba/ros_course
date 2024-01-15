#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def node1():
    rospy.init_node('node1', anonymous=True)
    node1 = rospy.Publisher('data_transfer_topic', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        number = int(input("Enter a number for node1: "))
        rospy.loginfo(f"Entered number for node1: {number}")
        node1.publish(number)
        rate.sleep()

if __name__ == '__main__':
    try:
        node1()
    except rospy.ROSInterruptException:
        pass
