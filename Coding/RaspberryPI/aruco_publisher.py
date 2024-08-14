#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
import cv2
from cv2 import aruco
from std_msgs.msg import Int32MultiArray

flag = 0
rooms = []


class ArucoPublisher:
    def __init__(self):
        self.marker_pub = rospy.Publisher("/aruco_marker_flag", Int8, queue_size=10)
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def run(self):
        global flag
        global rooms
        rate = rospy.Rate(4)  # 4Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture frame")
                break

            # Rotate the screen
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Convert frame to grayscale for marker detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            try:
                # Detect markers
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict,
                                                                      parameters=self.parameters)
                rospy.loginfo("Flag value: %s", flag)
                marker_flag = 0  # Default publish value

                if ids is not None:
                    # Draw detected markers on the frame
                    aruco.drawDetectedMarkers(frame, corners, ids)

                    # Check if any of the detected marker IDs are in the rooms array
                    found_in_rooms = False
                    for marker_id in ids:
                        if marker_id[0] in rooms:
                            found_in_rooms = True
                            break

                    if found_in_rooms:
                        marker_flag = 1
                    else:
                        marker_flag = 2

                self.marker_pub.publish(marker_flag)

            except Exception as e:
                rospy.logerr("Error: {}".format(e))

            # Display the frame
            cv2.imshow('Aruco Detection', frame)

            # Check for key press to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        # Release the capture and close all OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()


def callback(msg):
    global flag
    # Process the received image data here
    rospy.loginfo("Received image from camera")
    flag = msg.data
    if flag == 1:
        ArucoPublisher().run()

def callback2(msg):
    global rooms
    # Update the rooms array with the received numbers
    rospy.loginfo("Received numbers: %s", msg.data)
    rooms = msg.data


if __name__ == "__main__":
    rospy.init_node('aruco_publisher', anonymous=True)
    rospy.Subscriber('CameraCheck', Int8, callback)
    rospy.Subscriber('number_array', Int32MultiArray, callback2)
    rospy.spin()
