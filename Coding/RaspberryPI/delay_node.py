#!/usr/bin/env python3

import rospy
import time

if __name__ == '__main__':
    rospy.init_node('delay')
    delay = rospy.get_param('~delay', 4)  # Default delay is 2 seconds
    rospy.sleep(delay)
