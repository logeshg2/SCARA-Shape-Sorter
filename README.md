# SCARA-Shape-Sorter
Shape sorter using SCARA robotic arm and Computer Vision

![scara](https://github.com/user-attachments/assets/9f92cea8-00a2-4f10-b470-e711481a97fb)

## Computer Vision:
### Object Detection:
YOLOv8 Object detecting algorith is used in this project. The YOLOv8s model is fine-tuned with the custom dataset, the custom trained weight file can be found in <a href="python_code/yolo_weights">here</a>. A total of 290 images were used in the training process. The dataset can be found <a href="https://app.roboflow.com/logws/shape-psldo-d7fmu/1">here</a>.

![detection](https://github.com/user-attachments/assets/427e22af-c331-4b4b-abff-b09a261c2fa0)

### Grasp Point and Inverse Projection:
