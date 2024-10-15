#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import message_filters
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

# Load ArUco dictionary and parameters
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

class TiagoPixelTo3D:
    def __init__(self, show_image=False):
        rospy.init_node('vision', anonymous=True)

        # Set up subscribers
        self.rgb_sub = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/xtion/depth_registered/image_raw', Image)
        self.camera_info_sub = rospy.Subscriber('/xtion/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Approximate time synchronizer for RGB and Depth
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # Publisher for ArUco marker transformed positions and IDs
        self.marker_pub = rospy.Publisher('/vision/aruco_marker', PointStamped, queue_size=10)
        self.id_pub = rospy.Publisher('/vision/aruco_marker_id', Int32, queue_size=10)

        # Transformation buffer and listener for TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Store the CameraInfo data
        self.camera_info_received = False
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set the show_image option
        self.show_image = show_image

        # Create display windows only if show_image is True
        if self.show_image:
            cv2.namedWindow("RGB Image")
            cv2.namedWindow("Depth Image")

    def camera_info_callback(self, camera_info_msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            self.camera_info_received = True

    def image_callback(self, rgb_image_msg, depth_image_msg):
        if not self.camera_info_received:
            rospy.logwarn("Camera info not received yet.")
            return

        # Convert the RGB image to an OpenCV image
        self.cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding='bgr8')
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

        # Normalize depth image for visualization
        cv_depth_image_normalized = cv2.normalize(cv_depth_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_depth_image_normalized = np.uint8(cv_depth_image_normalized)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(self.cv_rgb_image, ARUCO_DICT, parameters=ARUCO_PARAMS)

        # If markers are detected
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.08, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                cv2.aruco.drawDetectedMarkers(self.cv_rgb_image, corners, ids)
                cv2.aruco.drawAxis(self.cv_rgb_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                # Get the translation vector (tvecs) in the camera frame (camera_optical_frame)
                marker_position_camera = tvecs[i].flatten()

                # Transform the position directly to base_footprint
                transformed_position_base, euler_angles = self.transform_to_base_footprint(marker_position_camera, rgb_image_msg.header.stamp)

                if transformed_position_base:
                    # Log and publish the transformed position and marker ID
                    rospy.loginfo(f"Transformed position in base_footprint: {transformed_position_base}")
                    rospy.loginfo(f"Yaw: {euler_angles[0]}, Pitch: {euler_angles[1]}, Roll: {euler_angles[2]}")
                    
                    # Publish the transformed position as PointStamped
                    point_msg = PointStamped()
                    point_msg.header.stamp = rgb_image_msg.header.stamp
                    point_msg.header.frame_id = 'base_footprint'
                    point_msg.point.x = transformed_position_base[0]
                    point_msg.point.y = transformed_position_base[1]
                    point_msg.point.z = transformed_position_base[2]
                    self.marker_pub.publish(point_msg)

                    # Publish the marker ID
                    self.id_pub.publish(ids[i][0])

        # Show the RGB and depth images only if show_image is True
        if self.show_image:
            cv2.imshow("RGB Image", self.cv_rgb_image)
            cv2.imshow("Depth Image", cv_depth_image_normalized)
            cv2.waitKey(1)

    def transform_to_base_footprint(self, marker_position, stamp):
        try:
            # Create a PointStamped message for the marker's position in camera_optical_frame
            point_camera_optical = PointStamped()
            point_camera_optical.header.frame_id = 'xtion_optical_frame'
            point_camera_optical.header.stamp = stamp
            point_camera_optical.point.x = marker_position[0]
            point_camera_optical.point.y = marker_position[1]
            point_camera_optical.point.z = marker_position[2]

            # Get the full transform from camera_optical_frame to base_footprint
            transform = self.tf_buffer.lookup_transform('base_footprint', 'xtion_optical_frame', stamp, rospy.Duration(1.0))

            # Transform the point to base_footprint
            transformed_point = tf2_geometry_msgs.do_transform_point(point_camera_optical, transform)

            # Extract the translation part
            transformed_position = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]

            # Extract the rotation as a quaternion
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

            return transformed_position, (yaw, pitch, roll)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None, None


if __name__ == '__main__':
    try:
        # Argument to show images or not
        show_image = rospy.get_param("~show_image", False)
        processor = TiagoPixelTo3D(show_image=show_image)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
