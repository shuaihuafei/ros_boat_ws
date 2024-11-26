#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64  # Use Float64 to publish only the angle
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

# Load the YOLO model   yolo11n.pt best
model = YOLO(
    "/home/shuai/ros_boat_ws/src/cv/ultralytics/runs/detect/train12/weights/best.pt"
)
# model = YOLO("/home/shuai/ros_boat_ws/src/cv/ultralytics/yolo11n.pt")

# Initialize CvBridge
bridge = CvBridge()

# Camera intrinsic parameters (fx is obtained from camera calibration)
fx = 500.0  # Example focal length in pixels (replace with actual calibrated value)

# Scaling factor for enlarging the displayed image
scale_factor = 2.0  # 2x zoom (you can change this value for different scaling levels)

# Initialize the angle publisher
pub = rospy.Publisher("/object_angle", Float64, queue_size=10)


def image_callback(ros_image):
    try:
        # Convert the ROS Image message to a CV2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

        # Get the image dimensions (height, width)
        image_height, image_width = cv_image.shape[:2]
        cx = image_width / 2  # Image center x

        # Run YOLO prediction on the image
        results = model.predict(
            source=cv_image, show=False, classes=[0]
        )  # Detect class 0

        # Loop through the results
        for result in results:
            boxes = result.boxes  # YOLO bounding boxes

            for box in boxes:
                # Extract the bounding box coordinates (x1, y1, x2, y2)
                x1, y1, x2, y2 = box.xyxy[0]  # Box coordinates in pixel space
                # Calculate the center of the bounding box
                box_center_x = (x1 + x2) / 2

                # Calculate the difference between box center and image center
                dx = box_center_x - cx

                # Calculate the angle offset in radians (for x-axis only)
                angle_x = math.atan(dx / fx)  # Horizontal angle

                # Convert angle to degrees
                angle_x_deg = math.degrees(angle_x)

                # Publish only the angle
                pub.publish(angle_x_deg)

                rospy.loginfo(f"Angle to object (x-axis): {angle_x_deg} degrees")

                # Prepare the text to overlay (angle)
                text = f"Angle: {angle_x_deg:.2f} deg"  # Replacing ° with 'deg'

                # Draw the bounding box
                cv2.rectangle(
                    cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2
                )

                # Put the angle information on the image (text in green)
                cv2.putText(
                    cv_image,
                    text,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        # Resize the image to the desired scale
        enlarged_image = cv2.resize(
            cv_image,
            None,
            fx=scale_factor,
            fy=scale_factor,
            interpolation=cv2.INTER_LINEAR,
        )

        # Display the results (bounding boxes, confidence scores, etc.)
        # result_img = results[0].plot()  # Plot the predictions on the image
        cv2.imshow("YOLOv11 Predictions (Zoomed)", enlarged_image)
        cv2.waitKey(1)  # Display the image window

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")


def main():
    # Initialize the ROS node
    rospy.init_node("boat_image_predictor", anonymous=True)

    # Subscribe to the image topic (replace 'camera/image' with your topic name)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
