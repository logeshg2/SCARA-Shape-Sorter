# SCARA-Shape-Sorter
Shape sorter using SCARA robotic arm and Computer Vision

![scara](https://github.com/user-attachments/assets/9f92cea8-00a2-4f10-b470-e711481a97fb)

## Computer Vision:
### Object Detection:
YOLOv8 Object detecting algorith is used in this project. The YOLOv8s model is fine-tuned with the custom dataset, the custom trained weight file can be found in <a href="python_code/yolo_weights">here</a>. A total of 290 images were used in the training process. The dataset can be found <a href="https://app.roboflow.com/logws/shape-psldo-d7fmu/1">here</a>.

<img src="https://github.com/user-attachments/assets/427e22af-c331-4b4b-abff-b09a261c2fa0"  width="40%" /> 

### Grasp Point and Inverse Projection:
Grasp point is calculated using the bounding box from the object detector. This grasp point is passed to the inverse projection equation for calculating the 3D point from the 2D image.
For 3D point in camera frame, depth information from the camera (Intel Realsense D435i depth camera) is used.

<img src="https://github.com/user-attachments/assets/5eeb14a9-ecf9-4508-87a3-50723fa4f9ef"  width="40%" />

### Eye-to-Hand Calibration:
The calculated 3D grasp point is in camera frame, so in order for the robot to reach, the point must be in robot frame. This transformation is done by finding the transformation from base to camera (through hand-eye calibration).
