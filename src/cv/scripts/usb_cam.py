#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def main():
    # Initialize the ROS node
    rospy.init_node("camera_publisher", anonymous=True)

    # Get the video device parameter from the launch file (default to 0)
    video_device = rospy.get_param("~video_device", 0)

    # Create a publisher for the camera image
    image_pub = rospy.Publisher("/camera_usb/color/image_raw", Image, queue_size=10)

    # Initialize CvBridge
    bridge = CvBridge()

    # Open the camera with the given video device index
    cap = cv2.VideoCapture(video_device)

    if not cap.isOpened():
        rospy.logerr(f"Could not open video device {video_device}")
        return

    rate = rospy.Rate(30)  # Set the publishing rate (10 Hz)

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            break

        # Convert the OpenCV image to ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the image
        image_pub.publish(ros_image)

        # Optional: Display the captured frame (for debugging)
        # cv2.imshow("Camera Feed", frame)
        # if cv2.waitKey(1) & 0xFF == ord("q"):
        #     break

        rate.sleep()

    # Release the camera and close any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
