#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
led_pin = 16
GPIO.setup(led_pin, GPIO.OUT)

# Callback function for ROS messages
def callback(data):
    if data.data == 1:
        GPIO.output(led_pin, GPIO.LOW)
        time.sleep(20)
    elif data.data == 0:
        GPIO.output(led_pin, GPIO.HIGH)

def listener():
    # Initialize ROS node
    rospy.init_node('led_control', anonymous=True)

    # Subscribe to ROS topic
    rospy.Subscriber("aruco_marker_flag", Int8, callback)

    # Spin to keep the node from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
            
    finally:
        # Clean up GPIO settings
        GPIO.cleanup()
