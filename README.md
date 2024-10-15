
# 🤖 Tiago Mobile Manipulator Simulation in Gazebo with ROS

This project simulates the **Tiago Mobile Manipulator** robot 🦾 in a **Gazebo** environment 🏙, integrated with **ROS**. The robot performs navigation, object detection using ArUco markers, and object manipulation using the **MoveIt** library 🛠️.

## Features 🌟

1. **Environment Setup:**
   - A simulated room with two tables 🛋️.
   - Household objects like a Coke can 🥤 and a biscuit box 🍪 placed on the middle table.

2. **Autonomous Navigation:**
   - The robot autonomously navigates from one corner of the room to the middle table using the ROS navigation stack.
   - Utilizes pre-mapped environments for navigation (done using the Tiago mapping package).

3. **Object Manipulation:**
   - Detects objects using ArUco markers for 6-DOF localization.
   - Manipulates objects using the MoveIt library with collision avoidance.
   - Uses the **Play Motion** package for predefined arm movements.

## Key ROS Packages Used 🛠️

1. **ROS Navigation Stack 🌍:**
   - Navigate the robot from one location to another.
   - Publishes goals for the robot's movement using `move_base`.

2. **MoveIt 🤹:**
   - Plans and executes arm movements for object manipulation.
   - Ensures collision avoidance with tables and objects using the planning scene interface.

3. **ArUco Marker Detection 👁️:**
   - Detects 6-DOF poses of objects on the table using ArUco markers.
   - Converts the object poses from the camera frame to the robot's `base_footprint` frame using TF transforms.

4. **Play Motion Package:**
   - Uses predefined motions like 'reach_max' to move the robot's arm to specific positions 🏹.

5. **Head Contro:**
   - Moves the robot's head down to focus on objects using a `point_head_action` goal.

## Getting Started 🚀

### Prerequisites 📦

- Install **ROS Noetic** 🧑‍💻 and **Gazebo**.
- Install Tiago packages following the [Tiago Tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials).

### Setup 🛠️

1. Download and unzip the project files.
2. Move the `tiago_task` folder to the `tiago_simulation` directory of your catkin workspace.
3. Move the pre-saved map folder `room` to `$HOME/.pal/tiago_maps/configurations/`.
4. Place the `room.world` file in `/tiago_public_ws/src/pal_gazebo_worlds/worlds/`.

### Running the Simulation ▶️

1. Launch the navigation system in a terminal:

   ```bash
   roslaunch tiago_task tiago_navigation.launch public_sim:=true world:=room map:=$HOME/.pal/tiago_maps/configurations/room end_effector:=pal-gripper
   ```

2. In another terminal, launch the task:

   ```bash
   roslaunch tiago_task task.launch
   ```

## Project Workflow 🧩

1. **Start Simulation:** The robot is placed in one corner of the room.
2. **Navigation to Middle Table:** The robot navigates autonomously using the ROS navigation stack to the middle table.
3. **Object Detection and Manipulation:**
   - The vision node detects ArUco markers and publishes object poses.
   - The arm control node subscribes to the vision node and sends the end-effector to grasp the objects.
4. **Moving Objects:** The robot picks up the objects from the middle table and places them on the corner table, ensuring collision avoidance with the environment.
## Video 🎬

The video is 5x in speed showcase the robot actions:

https://user-images.githubusercontent.com/0df31f60-6123-4dcf-8fb9-c4cf5e36caaa


## Screenshots 🖼️

1. **Robot in the Environment:**
   ![Tiago Robot](https://github.com/KiaRational/Tiago_Object_Manipulation_ROS/blob/main/Readme/Fig3_ReachingTable1.png)

2. **Object Detection with ArUco Markers:**
   ![Aruco Detection](https://github.com/KiaRational/Tiago_Object_Manipulation_ROS/blob/main/Readme/Fig4_Detection.png)

3. **Arm Manipulation:**
   ![Robot Frames](https://github.com/KiaRational/Tiago_Object_Manipulation_ROS/blob/main/Readme/Fig5_Robot_Vision_Transform.png?raw=true)

## References 📚

- [OpenCV Documentation](https://docs.opencv.org/) 🔍
- [ROS Tiago Tutorials](http://wiki.ros.org/Robots/TIAGo/Tutorials) 🤖
- [ROS Documentation](http://wiki.ros.org/) 📖
- [MoveIt Documentation](https://moveit.ros.org/documentation/) ✋

---

This version adds emojis for a more engaging presentation, indicating key features, workflow steps, and tools involved in the project.
