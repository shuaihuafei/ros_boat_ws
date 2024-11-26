#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64  # Use Float64 to publish only the angle
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import math
import time  # For FPS calculation
import os
import rospkg

# 获取功能包路径
rospack = rospkg.RosPack()
package_path = rospack.get_path("cv")  # 功能包的名称

# 模型路径
# model_relative_path = "ultralytics/yolo11n.pt"
# model_relative_path = "ultralytics/yolo11n_openvino_model/"
model_relative_path = "ultralytics/runs/detect/train15/weights/best.pt"
# model_relative_path = "ultralytics/runs/detect/train14/weights/best_openvino_model/"

# 组合完整路径
model_path = os.path.join(package_path, model_relative_path)
# Load the YOLO model
model = YOLO(model_path, task="detect")

# Initialize CvBridge
bridge = CvBridge()

# Camera intrinsic parameters (fx is obtained from camera calibration)
fx = 500.0  # Example focal length in pixels (replace with actual calibrated value)

# Scaling factor for enlarging the displayed image
scale_factor = 2.0  # 2x zoom (you can change this value for different scaling levels)

# Initialize publishers
angle_pub = rospy.Publisher("/yolo/boat_angle", Float64, queue_size=10)
image_pub = rospy.Publisher("/yolo/predict_boat", Image, queue_size=10)

# Variables for FPS calculation
prev_time = None
fps = 0.0


def image_callback(ros_image):
    global prev_time, fps

    try:
        # Start time for the current frame
        curr_time = time.time()

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

                # Extract class ID and confidence score
                class_id = int(box.cls[0])  # Class ID
                confidence = float(box.conf[0])  # Confidence score

                # Get class name (replace with your class list if needed)
                class_name = model.names[class_id]

                # Only process if the class name is "boat"
                if class_name == "boat":
                    # Calculate the center of the bounding box
                    box_center_x = (x1 + x2) / 2

                    # Calculate the difference between box center and image center
                    dx = box_center_x - cx

                    # Calculate the angle offset in radians (for x-axis only)
                    angle_x = math.atan(dx / fx)  # Horizontal angle

                    # Convert angle to degrees
                    angle_x_deg = math.degrees(angle_x)

                    # Publish only the angle
                    angle_pub.publish(angle_x_deg)

                    rospy.loginfo(
                        f"Detected {class_name} with confidence {confidence:.2f}"
                    )
                    rospy.loginfo(f"Angle to object (x-axis): {angle_x_deg} degrees")

                    # Prepare the text to overlay (class name and angle)
                    text = f"{class_name} ({confidence:.2f}) | Angle: {angle_x_deg:.2f} deg"

                    # Draw the bounding box
                    cv2.rectangle(
                        cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2
                    )

                    # Put the class name and angle information on the image
                    cv2.putText(
                        cv_image,
                        text,
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

        # Calculate FPS
        if prev_time is not None:
            time_diff = curr_time - prev_time
            fps = 1.0 / time_diff if time_diff > 0 else 0.0
        prev_time = curr_time

        # Overlay FPS on the image
        fps_text = f"FPS: {fps:.2f}"
        cv2.putText(
            cv_image,
            fps_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 0, 0),
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

        # Publish the processed image
        ros_processed_image = bridge.cv2_to_imgmsg(enlarged_image, encoding="bgr8")
        image_pub.publish(ros_processed_image)

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge error: {e}")
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
