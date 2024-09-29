#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class NodeD:
    def __init__(self):
        self.pub = rospy.Publisher('outgoing_D', String, queue_size=10)
        self.sub = rospy.Subscriber('incoming_D', String, self.callback)

    def callback(self, msg):
        modified_msg = msg.data + "D"
        rospy.loginfo(f"Node D received and modified: {modified_msg}")
        self.pub.publish(modified_msg)

if __name__ == '__main__':
    rospy.init_node('node_D')
    node = NodeD()
    rospy.spin()
