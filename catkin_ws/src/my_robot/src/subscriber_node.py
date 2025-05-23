#!/usr/bin/env python3
import rospy
from my_robot.msg import message  # Импорт вашего сообщения

def callback(msg):
    rospy.loginfo(f"Received: time={msg.stamp} data={msg.data}")

def listener():
    rospy.init_node('message_listener')
    rospy.Subscriber("message_topic", message, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
