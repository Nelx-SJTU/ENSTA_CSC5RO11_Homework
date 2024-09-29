#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class NodeB:
    def __init__(self):
        self.pub = rospy.Publisher('outgoing_B', String, queue_size=10)
        self.sub = rospy.Subscriber('incoming_B', String, self.callback)

    def callback(self, msg):
        modified_msg = msg.data + "B"
        rospy.loginfo(f"Node B received and modified: {modified_msg}")
        self.pub.publish(modified_msg)

if __name__ == '__main__':
    rospy.init_node('node_B')
    node = NodeB()
    rospy.spin()
