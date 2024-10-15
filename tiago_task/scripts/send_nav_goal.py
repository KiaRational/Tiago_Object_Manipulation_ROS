#!/usr/bin/env python

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import moveit_commander
from tf.transformations import quaternion_from_euler
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

# Global variables to store marker data
marker_position = None
marker_id = None

# Gripper control function
def control_gripper(close=True):
    joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    if close:
        closed_positions = [0.015, 0.015]  # Positions to close the gripper
    else:
        closed_positions = [0.055, 0.055]  # Positions to open the gripper

    # Create action client to control the gripper
    client = actionlib.SimpleActionClient("gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for gripper_controller...")

    if not client.wait_for_server(rospy.Duration(5.0)):  # Wait for 5 seconds for the server
        rospy.logerr("Gripper controller is not available. Check if it is launched properly.")
        return False

    rospy.loginfo("...connected to gripper controller.")

    # Wait for joint states to be available
    rospy.wait_for_message("joint_states", JointState)

    # Create a joint trajectory message for the gripper
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = closed_positions
    trajectory.points[0].velocities = [0.0 for _ in joint_names]
    trajectory.points[0].accelerations = [0.0 for _ in joint_names]
    trajectory.points[0].time_from_start = rospy.Duration(2.0)

    # Send the goal to the gripper controller
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    rospy.loginfo("Closing gripper...")
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(5.0)):
        rospy.logerr("Failed to close the gripper within the expected time.")
        return False

    rospy.loginfo("Gripper closed.")
    return True

# Add collision objects to the planning scene
def add_collision_objects(scene,x,y,z):
    # Define a box (obstacle) in the planning scene
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_footprint"
    box_pose.pose.position.x = x  # Adjust position of the box
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z # Height of the obstacle
    box_pose.pose.orientation.w = 1.0

    box_size = (0.1, 0.1, 0.065)  # Define the size of the box (width, depth, height)
    rospy.sleep(1)

    # Add the box to the scene
    scene.add_box("obstacle_box", box_pose, box_size)

    table_pose = PoseStamped()
    table_pose.header.frame_id = "base_footprint"
    table_pose.pose.position.x = x + 0.20 # Adjust position of the box
    table_pose.pose.position.y = y 
    table_pose.pose.position.z = z - 0.13 # Height of the obstacle
    table_pose.pose.orientation.w = 1.0

    table_size = (0.8, 1.5, 0.05)  # Define the size of the box (width, depth, height)
    rospy.sleep(1)

    # Add the box to the scene
    scene.add_box("table", table_pose, table_size)

    rospy.loginfo("Added a collision box to the planning scene.")

def move_arm_torso(x, y, z, roll, pitch, yaw,cont_gripper=False, close=True, collision=True):
    # Initialize MoveIt! 
    moveit_commander.roscpp_initialize(sys.argv)

    group = moveit_commander.MoveGroupCommander("arm_torso")
    scene = PlanningSceneInterface()

    # Set the planning frame and end effector
    group.set_pose_reference_frame("base_footprint")
    group.set_end_effector_link("gripper_grasping_frame")
    # group.set_max_velocity_scaling_factor(0.5)
    # Add collision objects
    if collision:
        add_collision_objects(scene, x, y, z - 0.075)
    
    # Wait for the planning scene to update
    rospy.sleep(2)

    # Create the goal pose for the end-effector
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base_footprint"
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    # Convert roll, pitch, yaw to quaternion
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    # Set the goal pose and log the target
    group.set_pose_target(goal_pose)
    rospy.loginfo(f"Planning to move gripper_grasping_frame to: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")

    # Plan the motion
    plan = group.plan()

    # Check if the plan was successful
    if not plan:
        rospy.logerr("No plan found, aborting movement.")
        return False  # Early return if planning fails

    rospy.loginfo("Plan found. Executing the plan.")

    # Execute the plan
    group.go(wait=True)
    rospy.sleep(1)  # Wait for a short duration to allow motion to settle

    rospy.loginfo("Plan found. Executing the plan.")

    # Execute the plan
    group.go(wait=True)

    rospy.loginfo("Motion complete.")

    # Get and log the current position of the gripper
    gripper_pose = group.get_current_pose(end_effector_link="gripper_grasping_frame")
    rospy.loginfo(f"Gripper position with respect to base_footprint: {gripper_pose.pose}")
    rospy.sleep(2)  # Wait for a short duration to allow motion to settle

    # Close the gripper after reaching the goal pose
    if cont_gripper:
        if not control_gripper(close):
            rospy.logerr("Failed to close the gripper.")

    # Clear the target pose
    group.clear_pose_targets()
    

    return True


def move_to_goal(x, y, z, w):
    # Create a SimpleActionClient for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create a new goal to send to move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Use "map" as reference frame
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Set the goal position
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    # Set the goal orientation (quaternion)
    goal.target_pose.pose.orientation.w = w

    # Send the goal
    client.send_goal(goal)

    # Wait for the robot to reach the goal or fail
    finished_within_time = client.wait_for_result()

    # Get the result of the goal
    state = client.get_state()

    if finished_within_time and state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.loginfo("The robot failed to reach the goal.")
        return False

def play_motion(motion_name):
    # Create a SimpleActionClient for play_motion
    client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)

    rospy.loginfo("Waiting for play_motion action server...")
    client.wait_for_server()
    rospy.loginfo("...connected.")

    # Wait for joint states to be published
    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(3.0)

    rospy.loginfo(f"Executing play_motion: {motion_name}...")

    # Create a PlayMotionGoal and specify the motion name
    goal = PlayMotionGoal()
    goal.motion_name = motion_name
    goal.skip_planning = False

    # Send the goal to the play_motion action server
    client.send_goal(goal)
    
    # Wait for the result
    client.wait_for_result(rospy.Duration(10.0))
    
    rospy.loginfo(f"Motion '{motion_name}' executed.")

def move_head_down(z=0.9):
    # Create a SimpleActionClient for the head controller
    client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)

    # Wait for the head controller action server to start
    rospy.loginfo("Waiting for head controller action server...")
    client.wait_for_server()
    rospy.loginfo("...connected to head controller.")

    # Define the target point (look down in front of the robot)
    point = PointStamped()
    point.header.frame_id = "base_link"  # Reference frame for the target point
    point.header.stamp = rospy.Time.now()

    # Set the point coordinates in front and slightly down from the robot
    point.point.x = 1.0  # 1 meter in front of the robot
    point.point.y = 0.0  # directly in front
    point.point.z = z  # look slightly downward

    # Create a PointHeadGoal
    goal = PointHeadGoal()
    goal.target = point
    goal.pointing_frame = "xtion_rgb_optical_frame"  # Use the camera's optical frame
    goal.pointing_axis.x = 0.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 1.0
    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 0.25  # Set a slow head movement speed

    # Send the goal to the head controller
    rospy.loginfo("Sending goal to head controller to look down.")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
    rospy.loginfo("Head moved down.")

def marker_callback(msg):
    global marker_position, marker_id
    # Store the marker position and ID
    marker_position = (msg.point.x, msg.point.y, msg.point.z)
    # rospy.loginfo(f"Received Marker Position: {marker_position}")

def id_callback(msg):
    global marker_id
    marker_id = msg.data
    # rospy.loginfo(f"Received Marker ID: {marker_id}")

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('tiago_task_manager')
		
        # Subscribe to the marker position and ID topics
        rospy.Subscriber('/vision/aruco_marker', PointStamped, marker_callback)
        rospy.Subscriber('/vision/aruco_marker_id', Int32, id_callback)

        # play_motion('home')
        # moving to table 1
        if move_to_goal(3.5, -2.0, -0.7071, 0.7071):
            # If the robot reaches the goal, send the play_motion action
            play_motion('reach_max')
            if move_to_goal(3.5, -2.5, -0.7071, 0.7071):
                # After play motion, move head down
                move_head_down()
                # table 1 reached
                # vision data should receive
                
        if move_to_goal(3.5, -2.0, 0.7071, 0.7071):
            # If the robot reaches the goal, send the play_motion action
            play_motion('home')
        # Moving to Table 2
        if move_to_goal(5.9, 1.5, 0.7071, 0.7071):
            # If the robot reaches the goal, send the play_motion action
            play_motion('reach_max')
        if move_to_goal(5.9, 2.25, 0.7071, 0.7071):
            # After play motion, move head down
            move_head_down()
            # Table 2 reached
            rospy.loginfo("Table 2 reached. Ready to receive marker data.")

            # Here you can process the received marker data
            rospy.sleep(10)  # Wait for some time to collect data
            rospy.loginfo(f"Final Marker Position: {marker_position}, Marker ID: {marker_id}")
            last_marker = marker_position
            move_arm_torso(marker_position[0]-0.1,marker_position[1],marker_position[2]+0.14,0.0,0.0,0.0,False,False,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.14,0.0,0.0,0.0,True,True,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.19,0.0,0.0,0.0,False,True,False)
        if move_to_goal(3.3, -2.5, -0.7071, 0.7071):
            # After play motion, move head down
            move_head_down()
            # Table 2 reached
            rospy.loginfo("Table 1 reached. Ready to receive marker data.")
            # Here you can process the received marker data
            rospy.sleep(10)  # Wait for some time to collect data
            rospy.loginfo(f"Final Marker Position: {marker_position}, Marker ID: {marker_id}")
            last_marker = marker_position
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.14,0.0,0.0,0.0,True,False,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.35,0.0,0.0,0.0,False,False,False)
        if move_to_goal(6.3, 1.5, 0.7071, 0.7071):
            play_motion('reach_max')
        if move_to_goal(6.4, 2.3, 0.7071, 0.7071):
            # After play motion, move head down
            move_head_down()
            # Table 2 reached
            rospy.loginfo("Table 2 reached. Ready to receive marker data.")

            # Here you can process the received marker data
            rospy.sleep(10)  # Wait for some time to collect data
            rospy.loginfo(f"Final Marker Position: {marker_position}, Marker ID: {marker_id}")
            last_marker = marker_position
            move_arm_torso(marker_position[0]-0.1,marker_position[1],marker_position[2]+0.14,0.0,0.0,0.0,False,False,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.13,0.0,0.0,0.0,True,True,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.45,0.0,0.0,0.0,False,True,False)

        if move_to_goal(4.0, -2.5, -0.7071, 0.7071):
            # After play motion, move head down
            move_head_down()
            # Table 2 reached
            rospy.loginfo("Table 2 reached. Ready to receive marker data.")
            # Here you can process the received marker data
            rospy.sleep(10)  # Wait for some time to collect data
            rospy.loginfo(f"Final Marker Position: {marker_position}, Marker ID: {marker_id}")
            last_marker = marker_position
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.15,0.0,0.0,0.0,True,False,True)
            move_arm_torso(marker_position[0],marker_position[1],marker_position[2]+0.3,0.0,0.0,0.0,False,False,False)
        move_to_goal(0.0, 0.0, 0.0, 0.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation, PlayMotion, or Head Control interrupted.")
