from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO(
    "/home/shuai/ros_boat_ws/src/cv/ultralytics/runs/detect/train12/weights/best.pt"
)

# Export the model
model.export(format="openvino")  # creates 'yolov8n_openvino_model/'
