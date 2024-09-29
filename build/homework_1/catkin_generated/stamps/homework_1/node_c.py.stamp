#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class NodeC:
    def __init__(self):
        self.pub = rospy.Publisher('outgoing_C', String, queue_size=10)
        self.sub = rospy.Subscriber('incoming_C', String, self.callback)

    def callback(self, msg):
        modified_msg = msg.data + "C"
        rospy.loginfo(f"Node C received and modified: {modified_msg}")
        self.pub.publish(modified_msg)

if __name__ == '__main__':
    rospy.init_node('node_C')
    node = NodeC()
    rospy.spin()
