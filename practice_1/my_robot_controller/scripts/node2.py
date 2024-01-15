#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

node1_number = None
pub = rospy.Publisher('result_313315', Int32, queue_size=10)

def callback(data):
    global node1_number
    if node1_number is None:
        node1_number = data.data
    else:
        result = node1_number - data.data
        rospy.loginfo(f"Two numbers were obtained: from node1: {node1_number}, from node3: {data.data}. Result: {node1_number} - {data.data} = {result}")
        pub.publish(result)
        node1_number = None

def node2():
    rospy.init_node('node2', anonymous=True)
    rospy.Subscriber('data_transfer_topic', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        node2()
    except rospy.ROSInterruptException:
        pass
    