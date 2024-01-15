#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose


class TurtlePosePublisher():
    def __init__(self,
                 node_name: str,
                 turtle_name: str, 
                 pose_topic_name: str, 
                 rate: int):
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo("Node started.")
        rospy.on_shutdown(self.shutdown)  # Register handler to be called (self.shutdown()) when rospy process begins shutdown

        self.pub = rospy.Publisher(pose_topic_name, Pose, queue_size=1)
        self.sub = rospy.Subscriber(turtle_name + '/pose', Pose, self.pose_callback)
        
        self.rate = rospy.Rate(rate)
        self.cur_pose = (0, 0)

    def pose_callback(self, msg: Pose):
        self.cur_pose = (msg.x, msg.y)

    def spin(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"Current pose: ({self.cur_pose[0]}, {self.cur_pose[1]}).")
            self.pub.publish(Pose(x=self.cur_pose[0], y=self.cur_pose[1]))
            self.rate.sleep()
    
    def shutdown(self):
        self.pub.publish(Pose())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        pose_publisher = TurtlePosePublisher(node_name="turtle_pose_publisher",
                                          turtle_name="/ns1_313315/turtle1",
                                          pose_topic_name="turtle_pose_topic",
                                          rate=50)
        pose_publisher.spin()
    except rospy.ROSInterruptException:
        pass