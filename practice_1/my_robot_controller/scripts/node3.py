#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def node3():
    rospy.init_node('node3', anonymous=True)
    node3 = rospy.Publisher('data_transfer_topic', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        number = int(input("Enter a number for node3: "))
        rospy.loginfo(f"Entered number for node3: {number}")
        node3.publish(number)
        rate.sleep()

if __name__ == '__main__':
    try:
        node3()
    except rospy.ROSInterruptException:
        pass