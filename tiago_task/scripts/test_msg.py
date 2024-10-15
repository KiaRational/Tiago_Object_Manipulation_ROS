#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32

class MarkerSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('marker_subscriber', anonymous=True)

        # Initialize lists to store marker positions and IDs
        self.marker_positions = []
        self.marker_ids = []

        # Subscribe to the marker positions
        self.marker_sub = rospy.Subscriber('/vision/aruco_marker', PointStamped, self.marker_callback)
        # Subscribe to the marker IDs
        self.id_sub = rospy.Subscriber('/vision/aruco_marker_id', Int32, self.id_callback)

    def marker_callback(self, msg):
        # Save the marker position
        position = (msg.point.x, msg.point.y, msg.point.z)
        self.marker_positions.append(position)
        rospy.loginfo(f"Received Marker Position: {position}")

    def id_callback(self, msg):
        # Save the marker ID
        self.marker_ids.append(msg.data)
        rospy.loginfo(f"Received Marker ID: {msg.data}")

    def run(self):
        rospy.loginfo("Marker Subscriber is running.")
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = MarkerSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
