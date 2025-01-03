import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header,Int32, String
from geometry_msgs.msg import PoseStamped
from tf_transformations import *
from sklearn.cluster import KMeans
import pandas as pd
from scipy.interpolate import CubicSpline

from visualization_msgs.msg import Marker, MarkerArray


class PARKING(Node):
    def __init__(self):
        super().__init__("parking")
        qos_profile = QoSProfile(depth=10)
        self.sub_cone = self.create_subscription(
            PoseArray, "cone_pose_map", self.callback_cone, qos_profile
        )
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )
        
        # self.sub_flag = self.create_subscription(
        #     String, "flag", self.callback_flag, qos_profile
        # )
        
        # self.pub_gear = self.create_publisher(Int32, "gear", qos_profile)
        
        self.pub_path = self.create_publisher(Path, "path", qos_profile)
        
        self.path_timer = self.create_timer(0.5,self.path_publisher)
        
        self.marker_tiemr = self.create_timer(1,self.marker)

        self.pub_marker = self.create_publisher(MarkerArray, "cone_marker", qos_profile)

        self.declare_parameter("num_cones", 21)
        self.num_cones = self.get_parameter("num_cones").value

        self.marker_id = 1  # 마커 ID를 위한 초기 값
        self.cone = []
        self.j=0
        self.i = 1
        self.dis = []
        self.domain = []
        self.rotated_domain = []
        self.origin = np.array([])
        self.path_x = []
        self.path_y = []
        self.ds = 0.8
        self.path = []
        self.min_x = 0.0
        self.min_y = 0.0
        self.max_x = 0.0
        self.max_y = 0.0
        self.r = 6.03
        self.dangle = 0
        self.dlocal_x = 0
        self.dlocal_y = 0
        self.local_x = 0.0
        self.local_y = 0.0
        self.angle = 0
        # self.flag_ = 0
        # self.flag =""
        self._a,self._b,self._c = 0,0,0
        
    def marker(self):
        self.publish_markers()
        self.publish_markers_()
        
    def path_publisher(self):
        if self.path:
            self.publish_path()
            
    def callback_cone(self, msg):
        for pose_x, pose_y in [
            (pose.position.x, pose.position.y) for pose in msg.poses
        ]:
            if len(self.cone) < self.num_cones:
                p1 = (pose_x, pose_y)
                if self.detection_area(p1):
                    if not self.euclidean_duplicate(p1):
                        self.cone.append((pose_x, pose_y))
                        print("\n" * 4)
                        self.get_logger().info(f"Cone {self.cone} is {len(self.cone)}")
                        print("\n" * 4)
                    else:
                        continue
                else:
                    continue
            else:
                continue
            
    def callback_local(self, msg):
        if 3 < len(self.cone) <= self.num_cones:
                self.handle_localization(msg)
        else:
                                            # self.gear_publisher(2)
                                            # #global_path_publish _til_target_idx is_end. after end local_path will be published
            if len(self.cone) <= 3:
                self.handle_detection_localization(msg)

    def handle_localization(self, msg):
        self.local_x = msg.pose.pose.position.x
        self.local_y = msg.pose.pose.position.y
        quarternion = [
            0,
            0,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.angle = euler_from_quaternion(quarternion)
        self.origin = np.array([self.local_x, self.local_y])
        rotated_points = self.rotate_points(
            np.array(self.cone), self.angle, self.origin
        )
        self.indexing(rotated_points)

    def handle_detection_localization(self, msg):
        self.dlocal_x = msg.pose.pose.position.x
        self.dlocal_y = msg.pose.pose.position.y
        quarternion = [
            0,
            0,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.dangle = euler_from_quaternion(quarternion)

    # def callback_flag(self,msg):
    #     self.flag = msg.data

    # def gear_publisher(self,gear):
    #     msg = Int32()
    #     msg. data = gear
    #     self.pub_gear.publish(msg)
    #     self.get_logger().info(f"\ngear = {gear}\n")
        
    def detection_area(self, p1):
        if len(self.cone) <= 3:
            if p1 != None:
                dis = m.sqrt(
                    (self.dlocal_x - p1[0]) ** 2 + (self.dlocal_y - p1[1]) ** 2
                )
                if dis <= self.r:
                    if len(self.cone) == 3:
                        self.compute_detection_area()
                        return True
                    return True
                else:
                    return False

        else:
            p1_ = self.rotate_points(
                p1, self._c, np.array([self._a, self._b])
            )
            if self.min_x < p1_[0] < self.max_x and self.min_y < p1_[1] < self.max_y:
                return True
            return False

    def compute_detection_area(self):
        dis = []
        self._a,self._b,self._c =self.dlocal_x,self.dlocal_y,self.dangle
        for i in range(3):
            dis.append(
                m.sqrt(
                    (self.dlocal_x - self.cone[i][0]) ** 2
                    + (self.dlocal_y - self.cone[i][1]) ** 2
                )
            )
        low_dis_idx = dis.index(min(dis))
        low_dis_cone = self.cone[low_dis_idx]
        self.low_dis_rot_point = self.rotate_points(
            low_dis_cone,
            self.dangle,
            np.array([self.dlocal_x, self.dlocal_y]),
        )
        self.min_x = self.low_dis_rot_point[0] - 1
        self.max_x = self.low_dis_rot_point[0] + 15
        self.min_y = self.low_dis_rot_point[1] - 3.8
        self.max_y = self.low_dis_rot_point[1] + 1

    def indexing(self, data):
        # Cluster data based on y values only
        y_values = data[:, 1].reshape(-1, 1)

        # Fit KMeans with 3 clusters
        kmeans_y = KMeans(n_clusters=3, random_state=42)
        kmeans_y.fit(y_values)

        # Get cluster labels
        labels_y = kmeans_y.labels_
        cluster_means = np.array([y_values[labels_y == i].mean() for i in range(3)])
        sorted_cluster_indices = np.argsort(cluster_means)
        label_mapping = {
            old_label: new_label
            for new_label, old_label in enumerate(sorted_cluster_indices)
        }
        sorted_labels_y = np.array([label_mapping[label] for label in labels_y])

        # Add sorted cluster labels to the original data
        clustered_data_y_sorted = np.hstack((data, sorted_labels_y.reshape(-1, 1)))
        
        # Convert to DataFrame for better display
        clustered_data_y_sorted_df = pd.DataFrame(
            clustered_data_y_sorted, columns=["X", "Y", "Cluster"]
        )

        # Convert each cluster to a sorted list of tuples
        cluster_0 = (
            clustered_data_y_sorted_df[clustered_data_y_sorted_df["Cluster"] == 0][
                ["X", "Y"]
            ]
            .sort_values(by="X")
            .values.tolist()
        )
        cluster_1 = (
            clustered_data_y_sorted_df[clustered_data_y_sorted_df["Cluster"] == 1][
                ["X", "Y"]
            ]
            .sort_values(by="X")
            .values.tolist()
        )
        cluster_2 = (
            clustered_data_y_sorted_df[clustered_data_y_sorted_df["Cluster"] == 2][
                ["X", "Y"]
            ]
            .sort_values(by="X")
            .values.tolist()
        )
        if len(cluster_2) >= 2:
            for i in range(len(cluster_2) - 1):
                self.dis.append(cluster_2[i + 1][0] - cluster_2[i][0])
            max_dis_idx = self.dis.index(max(self.dis))
            if max(self.dis) > 4:
                if max_dis_idx == 0:
                    self.get_logger().info("case1")
                    alpha = abs(self.local_y - cluster_2[3][1])
                    self.domain.extend(
                        [
                            (cluster_1[0][0] + 1, cluster_1[0][1]),
                            
                            
                            (
                                cluster_2[2][0] - (alpha - 0.08) - 1.25,
                                cluster_2[2][1] - 1.15,
                            ),
                            
                            (
                                cluster_2[2][0] - (alpha - 0.08) - 0.75,
                                self.local_y - 0.08 - (alpha - 0.08),
                            ),
                            (
                                cluster_2[2][0] - 0.5 - (alpha - 0.08) / 2,
                                self.local_y
                                - 0.08
                                - (1 - m.sqrt(3) / 2) * (alpha - 0.08)
                                - 0.25,
                            ),
                            
                            
                            (cluster_2[2][0], self.local_y - 0.08),
                            (cluster_2[3][0], self.local_y),
                            (self.local_x + 2, self.local_y),
                        ]
                    )
                    points = np.array(self.domain)
                    self.rotated_domain = self.rotate_points(points, -self.angle, self.origin)
                    self.path_x = self.rotated_domain[:, 0]
                    self.path_y = self.rotated_domain[:, 1]
                    self.interpolate_path()

                elif max_dis_idx == 4:
                    self.get_logger().info("case2")
                    
                    alpha = self.local_y - cluster_2[6][1]
                    
                    self.domain.extend(
                        [
                            (cluster_1[1][0] + 1, cluster_1[1][1]),
                            
                            
                            (
                                cluster_2[6][0] - (alpha - 0.08) - 1.25,
                                cluster_2[6][1] - 1.15,
                            ),
                            
                            (
                                cluster_2[6][0] - (alpha - 0.08) - 0.75,
                                self.local_y - 0.08 - (alpha - 0.08),
                            ),
                            (
                                cluster_2[6][0] - 0.5 - (alpha - 0.08) / 2,
                                self.local_y
                                - 0.08
                                - (1 - m.cos(m.pi/6)) * (alpha - 0.08)
                                - 0.25,
                            ),
                            
                            
                            (cluster_2[6][0], self.local_y - 0.08),
                            (cluster_2[7][0], self.local_y),
                            (self.local_x + 2, self.local_y),
                        ]
                    )
                    points = np.array(self.domain)
                    self.rotated_domain = self.rotate_points(points, -self.angle, self.origin)
                    self.path_x = self.rotated_domain[:, 0]
                    self.path_y = self.rotated_domain[:, 1]
                    self.interpolate_path()

                elif max_dis_idx == 8:
                    self.get_logger().info("case3")
                    alpha = abs(self.local_y - cluster_2[4][1])
                    self.domain.extend(
                        [
                            (cluster_1[2][0] + 1, cluster_1[2][1]),
                            
                            
                            (
                                cluster_2[9][0]+1.25 - (alpha - 0.08) - 1.25,
                                cluster_2[9][1] - 1.15,
                            ),
                            
                            (
                                cluster_2[9][0]+1.25 - (alpha - 0.08) - 0.75,
                                self.local_y - 0.08 - (alpha - 0.08),
                            ),
                            (
                                cluster_2[9][0] + 1.25 - 0.5 - (alpha - 0.08) / 2,
                                self.local_y
                                - 0.08
                                - (1 - m.sqrt(3) / 2) * (alpha - 0.08)
                                - 0.25,
                            ),
                            
                            
                            (cluster_2[9][0] +1.25, self.local_y - 0.08),
                            (cluster_2[9][0], self.local_y),
                            (self.local_x + 2, self.local_y),
                        ]
                    )
                    points = np.array(self.domain)
                    self.rotated_domain = self.rotate_points(points, -self.angle, self.origin)
                    self.path_x = self.rotated_domain[:, 0]
                    self.path_y = self.rotated_domain[:, 1]
                    self.interpolate_path_()
                else:
                    self.get_logger().fatal("prediction_error")
            
    def interpolate_path(self):
        x = np.array(self.path_x)
        y = np.array(self.path_y)
        dx_ = np.diff(x)
        dy_ = np.diff(y)
        ds =np.sqrt(dx_**2+dy_**2)
        s = np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")
            
            self.narrow = int(s[-1] / self.ds)
            self.get_logger().info(f"path:{self.narrow}. length = {(self.narrow-1) * 0.8} ")
            
            
            s_new = np.linspace(s[0], s[-1], 150)
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
            for i in range(len(yaw_new)):
                dyaw_new = []
                dyaw_new.append(yaw_new[i+1] - yaw_new[i])
            print(dyaw_new)
            dyaw_new = np.array(dyaw_new)
            print(dyaw_new)
            a=dyaw_new > 0.233 # 13.4(deg)
            print(any(a))
            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
        except Exception as e:
            self.get_logger().error(
                f"An error occurred during spline interpolation: {e}"
            )

    def publish_path(self):
        self.publish_markers()
        self.publish_markers_()
        self.publish_domain()
        # self.gear_publisher(0)
        
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for x, y, steer in self.path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            quaternion = quaternion_from_euler(0, 0, steer)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path.poses.append(pose)
        self.pub_path.publish(path)

    def rotate_points(self, points, angle, origin):
        angle_radians = -angle
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        # Translate points to origin
        translated_points = points - origin
        # Rotate points
        rotated_points = np.dot(translated_points, rotation_matrix.T)
        # Translate points back
        rotated_points += origin
        return rotated_points

    def euclidean_duplicate(self, p1):
        threshold = 0.8
        for p2 in self.cone:
            distance = m.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            if distance <= threshold:
                return True
        return False

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.cone):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cone"
            marker.id = self.marker_id + i  # 고유한 마커 ID 설정
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.duration.Duration(
                seconds=0
            ).to_msg()  # 무한 수명 설정

            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)

    def publish_markers_(self):
        marker_array = MarkerArray()

        area = [
            (x, y) for x in [self.min_x, self.max_x] for y in [self.min_y, self.max_y]
        ]
        rotated_area = self.rotate_points(
            np.array(area), -self.dangle, np.array([self.dlocal_x, self.dlocal_y])
        )

        for i, (x, y) in enumerate(rotated_area):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "area"
            marker.id = self.marker_id + i + 18  # 고유한 마커 ID 설정
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.duration.Duration(
                seconds=0
            ).to_msg()  # 무한 수명 설정

            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)

    def publish_domain(self):
        marker_array = MarkerArray()

        rotated_domain = self.rotate_points(
            np.array(self.domain), -self.angle, np.array([self.local_x, self.local_y])
        )

        for i, (x, y) in enumerate(rotated_domain):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "domain"
            marker.id = self.marker_id + i + 22  # 고유한 마커 ID 설정
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.lifetime = rclpy.duration.Duration(
                seconds=0
            ).to_msg()  # 무한 수명 설정

            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PARKING()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()