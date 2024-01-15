#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Controller():

    def __init__(self, 
                 controller_name: str, 
                 turtle_name: str, 
                 reference_topic: str, 
                 rate: int, 
                 factor: int):
        rospy.init_node(controller_name, anonymous=True)
        rospy.loginfo("Node started.")
        rospy.on_shutdown(self.shutdown)  # Register handler to be called when rospy process begins shutdown

        self.publisher = rospy.Publisher(turtle_name + '/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber(turtle_name + "/pose", Pose, self.turtle_pose_callback)
        rospy.Subscriber(reference_topic, Pose, self.reference_pose_callback)

        self.rate = rospy.Rate(rate)

        self.reference_x = 0
        self.reference_y = 0

        self.turtle_x = 0
        self.turtle_y = 0

        self.factor = factor
    
    def reference_pose_callback(self, msg: Pose):
        rospy.loginfo(f"Reference: ({msg.x}, {msg.y}).")
        self.reference_x, self.reference_y =  msg.x, msg.y

    def turtle_pose_callback(self, msg:Pose):
        rospy.loginfo(f"Current turtle pose: ({msg.x}, {msg.y}).")
        self.update_turtle_pose(msg.x, msg.y)

    def update_turtle_pose(self, x, y):
        self.turtle_x, self.turtle_y = x, y

    def form_control(self) -> Twist:
        # Simple P-regulator controller
        twist_msg = Twist()  
        # Form the control msg
        #  Structure of Twist() message:
        #  Vector3  linear
        #  Vector3  angular
        diff_x = self.reference_x - self.turtle_x
        diff_y = self.reference_y - self.turtle_y

        # if diff > 0.1:
        #     twist_msg.linear.x  = -0.5
        # elif self.x - self.target_x < -0.1:
        #     twist_msg.linear.x  = 0.5
        # else:
        #     twist_msg.linear.x  = 0.0

        twist_msg.linear.x = self.factor * diff_x
        twist_msg.linear.y = self.factor * diff_y
        return twist_msg

    def spin(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.form_control())
            self.rate.sleep()

    def shutdown(self):
        self.publisher.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        reference_topic = rospy.get_param('controller/reference_topic')

        controller = Controller(controller_name='controller',
                                turtle_name='turtle1',
                                reference_topic=reference_topic,
                                rate=50,
                                factor=1)
        controller.spin()
    except rospy.ROSInterruptException:
        pass