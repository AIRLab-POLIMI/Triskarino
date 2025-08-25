# Inference.py – Pose Classification and Feedback System

## Overview
`Inference.py` runs the pose classification system and creates a **real-time feedback view** for the operator to understand what the robot sees and which poses are being detected for each person in the camera's field of view.

The program:
- Recovers images in real-time from the robot by subscribing to the `/rpi_camera/image_raw/compressed` topic.
- Uses **YOLO11 with ByteTrack** for person detection, pose keypoint extraction, and tracking.
- Extracts features from each detected person and classifies their pose with an **XGBoost model**.
- Displays bounding boxes, keypoint body estimation (skeleton), and pose guessed in the feedback window for each person in the robot's field of view.
- Shows contextual information such as trigger messages if the robot is moving or being touched.
- Displays an estimation of direction for detected persons that are walking.

---

## Image Streaming and Processing
- Images are wirelessly streamed from the robot to the laptop using the **TCPROS** protocol.  
- This induces data transfer lag, which is detrimental to real-time feedback.  
- To mitigate lag:
  - **All frames** are displayed.
  - Only **1 out of 5 frames** is processed for pose classification.  

⚠️ Running inference directly on the robot was not feasible (caused lag and crashes). Running it remotely on a laptop is required, since the image feed needs to be streamed anyway for visualization.

---

## Pose Detection and Tracking
The program uses:
- **YOLO11** (`yolo11x-pose.pt`) for body and skeleton detection.
- **ByteTrack** (tuned via `bytetrack.yaml`) to:
  - Draw bounding boxes and skeletons.
  - Assign unique IDs to each detected person.
  - Maintain IDs consistently, even if someone temporarily leaves the frame or is occluded.

---

## Feature Extraction and Classification
For each detected person:
1. **Keypoints coordinates (X, Y)** on the 2D plane that is the frame, are extracted and normalized between 0 and 1.  
2. Additional features are computed from the extracted keypoint coordinates (angles of limbs and joints as well as some distances between keypoints).  
3. Depending on the trained **XGBoost (.xgb) model**, you may need to:
   - Comment/uncomment features in the dictionaries at the beginning of the code.
   - Comment/uncomment distance computations in the middle of the code.
4. Features are structured into a vector and passed into the model, which classifies the pose.  
5. The predicted pose is displayed inside the person’s bounding box in the feedback view.  

Currently, the program is tuned to work with the `Pose_Classification_Final.xgb` model trained on **4 classes**:
- Sitting (on the ground)  
- Standing  
- SittingInChair  
- Crawling  

---

## Additional Features
- **Robot movement and touch detection**  
  - Subscribes to movement and touch trigger topics.  
  - Displays a message in the feedback view when the robot is moving or being touched.  
  - Sensitivity of these triggers can be adjusted at the publishing nodes.  

- **Walking direction estimation**  
  - By comparing bounding box area, dimensions, and coordinates over time, the program attempts to detect if a person is walking and in which direction.  
  - Only triggers when a person is classified as standing **and** the robot is not moving.  

---

## Handling Edge Cases
Due to the robot’s **low-mounted camera** with a **poor angle of view**:
- When someone is standing or sitting very close, most of their body is out of frame.  
- Missing features make the model’s predictions unreliable.  

To address this, the program uses **hardline rules** (defined at the beginning of the code).  
Example: If ~70% of keypoints are missing but some upper body keypoints are visible, the program assumes the person is **sitting on the ground** or **lying on the floor**, overriding the model’s prediction.  

---

## Demo
A demo of the system working (recommended at 1.5× speed):  
👉 [Watch on YouTube](https://www.youtube.com/watch?v=8wJcS_wXHeY)

---

## Requirements (Recommended Working Setup)
To run `Inference.py` on your laptop, you will need:
- [Ultralytics YOLO11](https://github.com/ultralytics/ultralytics)
- ROS1 Noetic
- XGBoost
- OpenCV (cv2)
- Basic Python libraries such as `numpy` and `pandas` (see imports at the top of the code)

👉 On Ubuntu, it is strongly recommended to install YOLO, XGBoost, OpenCV, and other libraries via **pip** inside a **Miniconda virtual environment**.

---

## Running Without Native ROS1 (Using Docker)
If your operating system does not support ROS1 Noetic (e.g., Ubuntu 24.04):
1. Install Docker.  
2. Create or download a Docker image based on **Ubuntu 20.04**.  
3. Launch a container and install **ROS1 Noetic, YOLO11, and all other dependencies** inside it.  
4. Import the `Inference.py` program and the model into the container.  
5. Commit the container’s state to save your working setup.  

---

## ROS Network Setup

To stream the robot’s image feed to your laptop, you need to link your ROS client to the robot’s ROS master.  
⚠️ Both your laptop and the robot must be on the **same LAN**.

### Robot `.bashrc`
On the robot, add the following lines to `~/.bashrc`:

export ROS_MASTER_URI=http://192.168.104.103:11311  
export ROS_IP=192.168.104.103

- `ROS_MASTER_URI` points to the robot itself (the ROS master).  
- `ROS_IP` is the robot’s IP address on the LAN.  
This setup is usually the default unless the robot’s IP has been changed.

### Laptop `.bashrc`
On your laptop, add the following lines to `~/.bashrc`:

export ROS_MASTER_URI=http://192.168.104.103:11311  
export ROS_IP=192.168.103.102

- `ROS_MASTER_URI` points to the robot as the ROS master.  
- `ROS_IP` is your laptop’s IP address on the same LAN.  

### Finalizing the Setup
After editing `.bashrc` on both machines, make the changes permanent by running:

source ~/.bashrc

Now you should be able to launch `roscore` on the robot and see its nodes from your laptop. This confirms that the connection is working and you can subscribe to its topics.

---

## Final Step
Before running `Inference.py`, ensure that the **paths to your YOLO model** and **XGBoost model** inside the code are correctly set for your system.
