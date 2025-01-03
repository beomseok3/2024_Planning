#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
import numpy as np

from scipy.cluster.hierarchy import linkage, fcluster

class BezierCurveMaking(Node):

    def __init__(self):
        super().__init__('cone_pose_sub')

        qos_profile = QoSProfile(depth=10)

        self.sub_left = self.create_subscription(
            PoseArray,
            'cone_pose_map', 
            self.get_points_callback,
            qos_profile)
        
        self.pub_marker_array = self.create_publisher(
            MarkerArray,
            '/cone_points',
            qos_profile)

        self.pub_closest_point = self.create_publisher(
            Point,
            "/closest_point",
            qos_profile
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            "localization/kinematic_state",
            self.odom_callback,
            qos_profile
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.cone_pose = []
        self.cluster_centers = []
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.num = 0
    def odom_callback(self, msg):
        if self.num == 0 :
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y

        self.get_logger().info(f"{self.odom_x},{self.odom_y}")

    def get_points_callback(self, data):
        self.cones = [(pose.position.x, pose.position.y) for pose in data.poses]
        for i in self.cones:
            self.cone_pose.append(i)

        if len(self.cone_pose) < 2:
            # Not enough points to cluster
            return

        coords = np.array(self.cone_pose)
        Z = linkage(coords, method='average')
        distance_threshold = 0.3
        clusters = fcluster(Z, distance_threshold, criterion='distance')

        unique_clusters = np.unique(clusters)
        cluster_centers = []
        for cluster_id in unique_clusters:
            cluster_points = coords[clusters == cluster_id]
            cluster_center = np.mean(cluster_points, axis=0)
            cluster_centers.append(cluster_center)

        self.cluster_centers = np.array(cluster_centers)

        self.publish_cone_data()

    def publish_cone_data(self):
        if len(self.cluster_centers) == 0:
            return

        marker_array = MarkerArray()
        closest_distance = 2.0
        too_much = 1
        closest_index = -1

        #print(self.odom_x, self.odom_y)
        # Find the closest cluster center
        for i, (x, y) in enumerate(self.cluster_centers):
            distance = np.sqrt((x - self.odom_x)**2 + (y - self.odom_y)**2)
            if distance < closest_distance and distance > too_much:
                closest_distance = distance
                closest_index = i
                self.num = 1
        #print("distance = ", closest_distance)

        # Create markers
        for i, (x, y) in enumerate(self.cluster_centers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = -1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0

            if i == closest_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                point = Point()
                point.x = x
                point.y = y
                point.z = -1.0
                self.pub_closest_point.publish(point)
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)
        
        self.pub_marker_array.publish(marker_array)

    def timer_callback(self):
        self.publish_cone_data()

def main(args=None):
    rclpy.init(args=args)
    cone_pose_sub = BezierCurveMaking()
    rclpy.spin(cone_pose_sub)
    cone_pose_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
