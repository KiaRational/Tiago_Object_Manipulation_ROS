#!/usr/bin/env python

import sys
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import moveit_commander
from tf.transformations import quaternion_from_euler
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

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

    box_size = (0.05, 0.05, 0.035)  # Define the size of the box (width, depth, height)
    rospy.sleep(1)

    # Add the box to the scene
    scene.add_box("obstacle_box", box_pose, box_size)
    rospy.loginfo("Added a collision box to the planning scene.")

# Arm and torso control function with collision avoidance
def move_arm_torso(x, y, z, roll, pitch, yaw , close=True , collision=True):
    # Initialize MoveIt! and the ROS node
    rospy.init_node("plan_arm_torso_ik", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    group = moveit_commander.MoveGroupCommander("arm_torso")
    scene = PlanningSceneInterface()

    # Set the planning frame and end effector
    group.set_pose_reference_frame("base_footprint")
    group.set_end_effector_link("gripper_grasping_frame")

    # Add collision objects
    if collision:
        add_collision_objects(scene,x,y,z-0.08)
    
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
        raise Exception("No plan found")

    rospy.loginfo("Plan found. Executing the plan.")

    # Execute the plan
    group.go(wait=True)

    rospy.loginfo("Motion complete.")

    # Get and log the current position of the gripper
    gripper_pose = group.get_current_pose(end_effector_link="gripper_grasping_frame")
    rospy.loginfo(f"Gripper position with respect to base_footprint: {gripper_pose.pose}")

    # Close the gripper after reaching the goal pose
    if not control_gripper(close):
        rospy.logerr("Failed to close the gripper.")

    # Shutdown MoveIt commander
    moveit_commander.roscpp_shutdown()

# Main function
if __name__ == "__main__":
    if len(sys.argv) < 7:
        rospy.loginfo("Usage: plan_arm_torso_ik x y z roll pitch yaw")
    else:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        roll = float(sys.argv[4])
        pitch = float(sys.argv[5])
        yaw = float(sys.argv[6])

        try:
            move_arm_torso(x, y, z, roll, pitch, yaw)
        except rospy.ROSInterruptException:
            pass
