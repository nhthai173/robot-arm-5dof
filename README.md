# 🤖 5DOF Robotic Arm Object Sorting System

This project features a **5 Degrees of Freedom (5DOF) robotic arm** designed to automatically **pick up objects** within its workspace and **sort them into three different boxes** based on predefined criteria (e.g. color, size, shape).

## 📌 Project Overview

- **Control System**: The robotic arm is controlled via Jetson Xavier NX, using information received from a remote computer.
- **Distributed Vision Processing**:  
  - Jetson captures live images from the camera and sends them to a remote computer over **ROS 2**.
  - The computer performs **object detection** and **coordinate extraction** using YOLOv8 Segmentation.
  - The computed object position is sent back to Jetson, which controls the robotic arm to **pick and place** the object into the appropriate box.
- **Object Sorting**: Based on detected object class or location, items are sorted into **three designated boxes**.
- **Automation**: The system runs fully autonomously after being launched via ROS 2 nodes.

## 🧰 Technologies Used

### 🤖 Framework & Middleware
- **ROS 2 (Robot Operating System 2)**
  - Handles communication between Jetson and the computer via ROS 2 topics and nodes.
  - Enables modular, distributed processing.

### 🧠 Processing Units
- **Jetson Xavier NX**
  - Captures image from camera
  - Controls the 5DOF robotic arm via UART
  - Subscribes to object position topics
  - Publishes camera image topics  
  <img width="500px" alt="Jetson Xavier NX" src="image/README/1744535870585.png">

- **Remote Computer (PC or Laptop)**
  - Subscribes to camera feed
  - Runs object detection using YOLOv8 Segmentation
  - Publishes detected object coordinates  

### 📸 Vision System
- **Logitech C505 USB Webcam**
  - Connected to Jetson, streams images into ROS 2  
  <img width="300px" alt="Logitech C505 Webcam" src="image/README/1744535943105.png">

### 🦾 Robotic Arm
- **5DOF Robotic Arm**
  - Handles object picking and sorting
  - Controlled via servo driver module and UART  
  <img width="500px" alt="5DoF Robot Arm" src="image/README/1744534565053.png">

### ⚙️ Servo Control
- **Servo Driver Module**
  - Communicates with Jetson via UART
  - Executes movement commands for the robotic arm  
  <img width="300px" alt="Servo Control" src="image/README/1744536147471.png">

### 🧠 AI Model
- **YOLOv8 Segmentation**
  - Runs on the remote computer
  - Detects and segments objects
  - Computes their positions relative to the workspace  

### 💻 Programming Language
- **Python**
  - Used to write ROS 2 nodes, detection scripts, and robotic control logic


## 🎯 Object Positioning in Workspace

To accurately control the robotic arm for object picking, we compute the **coordinates of each object relative to a known origin point** within the camera’s view.

### 📐 Coordinate System Setup

1. **Reference Points Definition**  
   We manually define three reference points in the workspace:
   - **Origin (Gốc)**: This is the fixed (0,0) point of the coordinate system.
   - **X-Axis Point (Den_ngang)**: Used to define the direction and scale of the X-axis.
   - **Y-Axis Point (Den_doc)**: Used to define the direction and scale of the Y-axis.

2. **Real-World Measurements**  
   - The real distance between the origin and the X-axis reference point is **950 mm**.
   - The real distance between the origin and the Y-axis reference point is also **950 mm**.
   - These distances are used to calculate a **pixel-to-millimeter ratio** for coordinate transformation.

### 🧮 Object Coordinate Calculation

1. **Camera captures an image** of the workspace from a top-down view.
2. **YOLOv8 Segmentation** detects the object and identifies its center point in **pixel coordinates**.
3. The script calculates the position of the object relative to the **origin**, using the reference axis points to:
   - Determine orientation (angle correction if necessary)
   - Convert pixel distance into millimeters

### 🖼️ Example (Image Below)

The image below illustrates:
- The **reference points** (`Goc`, `Den_ngang`, `Den_doc`) with lines indicating axis directions.
- A detected object (`Vang`) with a blue dot at its center.
- Distance measurements (in mm) from the origin to each detected point.

![Workspace coordinate calculation](image/README/1744536700048.png)