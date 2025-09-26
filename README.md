# SPARC 🤖  
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)  ![Python 3.10](https://img.shields.io/badge/Python-3.10-green)  ![MoveIt](https://img.shields.io/badge/MoveIt-Enabled-orange)  ![YOLOv8](https://img.shields.io/badge/YOLOv8-OBB-red)  

**Smart Pick-n-Place with AI Robotics and Chatbot**  

![SPARC Demo](docs/PickPlace.gif)  

---

## 🌟 Overview  
SPARC is an intelligent **pick-and-place robotic system** built on **ROS2 Humble**, **Isaac Sim**, and **MoveIt**. It integrates:  
- **YOLOv8-OBB object detection** for real-time perception  
- **Franka Emika Panda / UR5 robot arms** for precise manipulation  
- **Chatbot interface** powered by **Ollama LLMs** for natural language control  

This allows users to **command the robot in plain English**, enabling smart and interactive task execution in simulation.  

---

## 🚀 Features  
- 🔍 Real-time object detection (YOLOv8 Oriented Bounding Boxes)  
- 🦾 Robotic arm control using MoveIt + ROS2  
- 🧠 Natural language chatbot interface (Ollama)  
- 🖼️ Camera calibration from Isaac Sim (fx, fy, cx, cy extraction)  
- 🛠️ Compatible with both **Franka Panda** and **UR5** arms  
- 🎯 Target point publishing for perception → planning integration  

---
## 📦 Dependencies  
- **ROS2 Humble**  
- **MoveIt** (`ros-humble-moveit-py`, `moveit` package)  
- **UR5 MoveIt config**  
- **Ultralytics (YOLOv8)**  
- **ROS2 Control + Controllers**  
  - `ros-humble-ros2-control`  
  - `ros-humble-ros2-controllers`  
  - `ros-humble-gripper-controllers`  
- **Ollama** (for chatbot LLMs)  
- **Conda environments** 

---

## ⚙️ Setup

1. Create a new **Conda environment** with Python 3.10  
2. Install dependencies from `requirements.txt`:  
   ```bash
   pip install -r requirements.txt
   ```

---

## 🤖 Running the Demo  

1. Open **Isaac Sim**  
2. Load the simulation file: `SPARC.usd`  
3. Start the simulation  
4. Clone this repository:  
   ```bash
   git clone https://github.com/sahilrajpurkar03/nlp-pnp-robotic-arm.git
   ```
5. Activate the Conda environment  
6. Navigate into the project folder:  
   ```bash
   cd nlp-pnp-robotic-arm
   ```
7. Launch the main script:  
   ```bash
   ./main_launch.sh
   ```

This will start the **chatbot interface**. You can then send natural language commands like:  
👉 *"Pick the red cube"*  

SPARC will handle the rest automatically 🚀  

---

## 🛠️ Debugging  

### Panda Arm + Motion Planning  
```bash
ros2 launch panda_moveit_config demo.launch.py
ros2 run panda_moveit_config move_arm_to_xyz 0.0 0.0 0.0
```

### UR5 Arm Demo  
```bash
cd ~/Isaac_Project/pickPlaceChatMoveitBot_ws/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch ur5_moveit_config demo.launch.py
ros2 run ur5_moveit_config ur5_pick_place_cpp_r
```

### Object Detection (YOLOv8-OBB)  
```bash
ros2 run yolov8obb_object_detection yolov8_obb_publisher
ros2 run yolov8obb_object_detection yolov8_obb_subscriber
```

### Chatbot Control  
```bash
cd ~/Isaac_Project/pickPlaceChatMoveitBot_ws/
conda activate ros2_humble_py310
source /opt/ros/humble/setup.bash
source install/setup.bash
./pick_place_chatbot_ui/launch.sh
```

### Check Published Target Points  
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /target_point
```

## 🎯 Competition Relevance  
SPARC demonstrates **seamless integration of AI and Robotics**, showcasing how natural language can be used to **control intelligent robotic arms** for pick-and-place tasks. This bridges perception, planning, and interaction—making robots more **intuitive, adaptable, and human-friendly**.  

---

## 📹 Demo Video  
👉 [Add your demo video link here]  

---
