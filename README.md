# 🐢 ROS2 Turtlesim Hunter Project

## 📌 Project Overview

This project demonstrates a **ROS2-based autonomous behavior** where a turtle continuously chases a **randomly spawned target turtle** inside the `turtlesim` simulator.

The goal of this project is to understand and practice **core ROS2 concepts** such as nodes, topics, services, velocity control, and simulation-based debugging, while building a simple but complete autonomous system.

---

## 🎯 What This Project Does

* Spawns a target turtle at a **random position**
* Subscribes to the pose of the main turtle
* Computes distance and angle to the target
* Publishes velocity commands so the turtle **chases the target autonomously**
* Uses proportional control for smooth motion

---

## 🧠 Core Concepts Used

* ROS2 Nodes (`rclpy`)
* Publishers & Subscribers
* ROS2 Services (`/spawn`)
* Velocity control using `geometry_msgs/Twist`
* Real-time feedback from `turtlesim/Pose`
* Simulation-based testing
* Git & GitHub workflow

---

## 🛠 Technologies & Tools

* **ROS2**
* **Python**
* **turtlesim**
* **rclpy**
* **Git & GitHub**
* **Ubuntu Linux**

---

## 📂 Project Structure

```
ros2-turtlesim-hunter/
├── turtlesim_hunter/
│   ├── hunter_node.py
│   └── __init__.py
├── resource/
├── package.xml
├── setup.py
├── setup.cfg
├── .gitignore
├── LICENSE
└── README.md
```

---

## ▶️ How to Run the Project

### 1️⃣ Clone the repository

```bash
git clone https://github.com/Psyduck-svg/ros2-turtlesim-hunter.git
```

### 2️⃣ Move into your ROS2 workspace

```bash
mv ros2-turtlesim-hunter ~/ros2_ws/src/
```

### 3️⃣ Build the workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4️⃣ Run turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### 5️⃣ Run the hunter node

```bash
ros2 run turtlesim_hunter hunter
```

✅ You should now see the turtle autonomously chasing the randomly spawned target.

---

## ❓ Questions & Doubts Solved During This Project

### 🔹 Why does GitHub reject password authentication?

GitHub no longer supports password authentication for Git operations.
A **Personal Access Token (PAT)** must be generated and used instead.

---

### 🔹 Why did `git push` get rejected with “fetch first”?

The GitHub repository already contained files (`README`, `LICENSE`) created online.
This required pulling remote changes first using:

```bash
git pull origin main --allow-unrelated-histories
```

---

### 🔹 Why is `.gitignore` important?

ROS2 generates build artifacts (`build/`, `install/`, `log/`) that should not be pushed to GitHub.
Ignoring them keeps the repository clean and professional.

---

### 🔹 Why add an MIT License?

Adding a license:

* Allows others to legally view and reuse the code
* Signals open-source awareness
* Is expected in professional robotics repositories

---

### 🔹 Why does the turtle overshoot sometimes?

This happens due to proportional control gains.
The project intentionally keeps control simple to highlight:

* Feedback-based motion
* Real-time behavior tuning
* Simulation debugging

---

## 📘 Key Learnings

* How ROS2 nodes communicate using topics and services
* How to control robot motion using velocity commands
* How to debug autonomous behavior in simulation
* How to structure and publish a ROS2 project on GitHub
* How to handle real Git & GitHub workflow issues

---

## 🚀 Future Improvements

* Respawn target after capture
* Add launch files
* Add PID control instead of proportional control
* Support multiple target turtles
* Visualize path or distance

---

## 👩‍💻 Author

**Vaishali Patel**
Robotics Enthusiast | ROS2 | Autonomous Systems

