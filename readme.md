

# 🦾 Surgical Robotic Arm - 
![Banner](https://img.shields.io/badge/Surgical_Robotic_Arm-Robosoc-blue?style=for-the-badge&logo=ros)

A modular and extensible 6-DOF robotic arm project powered by **ROS2 Humble**, designed for surgical simulation, precision control, and inverse kinematics experiments. Built to run in simulation with room for real hardware extension.

![robotic-arm-banner](https://your-awesome-image-link.com/banner.png)

---

## 🚀 Features

✅ ROS2-based modular architecture  
✅ Simulated 6-DOF robotic arm using URDF  
✅ Basic inverse kinematics (IK) engine  
✅ Launchable in RViz with live joint state updates  
✅ Extensible for Gazebo simulation or real hardware  

---

## 🧠 Use Cases

- Simulate robotic arm motions for surgical precision  
- Run IK solutions and visualize results in RViz  
- Learn ROS2 workflows: URDF, launch, and Python nodes  
- Foundation for further integration: Gazebo, MoveIt, sensors, etc.

---

## 🛠️ Tech Stack

- **ROS2 Humble** 🐢
- **Python 3**
- **URDF / Xacro**
- **RViz2**
- **Gazebo, MoveIt2, TF2, hardware interfaces**

---

## 📁 Project Structure

```

surgical\_robotic\_arm/
├── launch/
│   └── display.launch.py        # Launch RViz with arm model
├── urdf/
│   └── robotic\_arm.urdf         # Basic robot structure
├── robotic\_arm/
│   ├── **init**.py
│   └── inverse\_kinematics.py    # IK logic placeholder
├── package.xml
├── setup.py
└── README.md

````

---

## ⚙️ Getting Started

### 1. Clone the Repo

```bash
cd ~/My_Lab/ROS/
git clone https://github.com/yourusername/surgical_robotic_arm.git
cd surgical_robotic_arm
````

### 2. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the Arm in RViz

```bash
ros2 launch robotic_arm display.launch.py
```

### 4. Run the Inverse Kinematics Node

```bash
ros2 run robotic_arm inverse_kinematics
```

---

## 🧪 Example IK Output

```bash
$ ros2 run robotic_arm inverse_kinematics
Running Inverse Kinematics...
Target: [0.2, 0.1, 0.3]
Calculated joint angles: [0.0, 0.5, -0.3, 0.1, 0.0, 0.2]
```

---

## 🎯 Next Goals

* [ ] Add proper joint chain URDF (6 DOF)
* [ ] Integrate with Gazebo for physical simulation
* [ ] Implement real IK using numerical solvers
* [ ] Extend to use MoveIt2 for motion planning
* [ ] Deploy on a physical arm with ROS2 hardware interfaces

---

## 🤝 Contributing

Pull requests are welcome! Open an issue for bug reports, feature ideas, or collab requests. Let's build the future of surgical robotics together 🤖❤️‍🩹

---


## ✨ Maintainer

**Mr Ranjan** – [LinkedIn](https://www.linkedin.com/in/fictech/) | [GitHub](https://github.com/AshishNith)

**Chirayu** – [LinkedIn](https://www.linkedin.com/in/chirayu-pandey-23bme033) | [GitHub](https://github.com/chirayupandey)

**Purushottam Singh** – [LinkedIn](https://www.linkedin.com/in/purushottam-singh-9303bb25a?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app) | [GitHub](https://github.com/)

Made with 🧠, 🛠️, and a dream to build real-world robots.


