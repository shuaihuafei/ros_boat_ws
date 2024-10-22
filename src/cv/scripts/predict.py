import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("src/cv/ultralytics/yolo11n.pt")

# Initialize CvBridge
bridge = CvBridge()


def image_callback(ros_image):
    try:
        # Convert the ROS Image message to a CV2 image
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

        # Run YOLO prediction on the image
        results = model(cv_image)

        # Display the results (bounding boxes, confidence scores, etc.)
        result_img = results[0].plot()  # Plot the predictions on the image
        cv2.imshow("YOLOv11 Predictions", result_img)
        cv2.waitKey(1)  # Display the image window

    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")


def main():
    # Initialize the ROS node
    rospy.init_node("yolo_image_predictor", anonymous=True)

    # Subscribe to the image topic (replace 'camera/image' with your topic name)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    main()
