#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from itertools import cycle


class TargetNode():
    def __init__(self, 
                 points: list(), 
                 target_node_name: str, 
                 target_topic_name: str, 
                 controlled_object_name: str,
                 epsilon: list(),
                 rate: int):
        rospy.init_node(target_node_name, anonymous=True)
        rospy.loginfo("Node started.")
        rospy.on_shutdown(self.shutdown)  # Register handler to be called (self.shutdown()) when rospy process begins shutdown

        self.pub = rospy.Publisher(target_topic_name, Pose, queue_size=1)
        self.sub = rospy.Subscriber(controlled_object_name + '/pose', Pose, self.pose_callback)
        
        self.rate = rospy.Rate(rate)
        self.cycle = cycle(points)
        self.epsilon = epsilon
        self.cur_pose = (0, 0)

    def pose_callback(self, msg: Pose):
        self.cur_pose = (msg.x, msg.y)

    def near_target(self, target) -> bool:
        return all((abs(self.cur_pose[i] - target[i]) < self.epsilon[i] for i in (0, 1)))

    def spin(self):
        for target in self.cycle:
            if rospy.is_shutdown():
                return
            rospy.loginfo(f"Current target: ({target[0]}, {target[1]}).")
            while not self.near_target(target):
                if rospy.is_shutdown():
                    return
                self.pub.publish(Pose(x=target[0], y=target[1]))
                self.rate.sleep()
    
    def shutdown(self):
        self.pub.publish(Pose())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        target_node = TargetNode(points=[(2, 3), (7, 1), (8, 4)], 
                                 target_node_name="target_node",
                                 target_topic_name="target_topic",
                                 controlled_object_name="/ns1_313315/turtle1",
                                 epsilon=(0.1, 0.1),
                                 rate=50)
        target_node.spin()
    except rospy.ROSInterruptException:
        pass