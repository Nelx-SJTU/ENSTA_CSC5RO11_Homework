#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class NodeA:
    def __init__(self):
        self.pub = rospy.Publisher('outgoing_A', String, queue_size=10)
        self.sub = rospy.Subscriber('incoming_A', String, self.callback)
        self.message = "Message from A"
        rospy.Timer(rospy.Duration(2), self.timer_callback)

    def callback(self, msg):
        rospy.loginfo(f"Node A received: {msg.data}")

    def timer_callback(self, event):
        rospy.loginfo("Node A sent: " + self.message)
        self.pub.publish(self.message)

if __name__ == '__main__':
    rospy.init_node('node_A')
    node = NodeA()
    rospy.spin()
