#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

class LaserLogger:
    def __init__(self):
        self.last_time = rospy.Time.now()
        self.log_interval = rospy.Duration(1.0)  # Интервал в 1 секунду

    def callback(self, msg):
        current_time = rospy.Time.now()
        if (current_time - self.last_time) >= self.log_interval:
            self.last_time = current_time
            rospy.loginfo("\n--- LaserScan Data (1 sec update) ---")
            rospy.loginfo(f"Frame ID: {msg.header.frame_id}")
            rospy.loginfo(f"Ranges[0]: {msg.ranges[0]:.2f} m")
            rospy.loginfo(f"Ranges[-1]: {msg.ranges[-1]:.2f} m")
            rospy.loginfo(f"Angle_min: {msg.angle_min} c")

def listener():
    rospy.init_node('laser_data_listener')
    logger = LaserLogger()
    rospy.Subscriber("scan", LaserScan, logger.callback)
    rospy.loginfo("LaserScan logger started (1 sec updates)...")
    rospy.spin()

if __name__ == '__main__':
    listener()