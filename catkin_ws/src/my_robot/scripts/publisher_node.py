#!/usr/bin/env python3
import rospy
from my_robot.msg import message
from std_msgs.msg import Time

def talker():
    pub = rospy.Publisher('message_topic', message, queue_size=10)
    rospy.init_node('message_talker')
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        msg = message()
        msg.stamp = rospy.Time.now()
        msg.data = 42
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
