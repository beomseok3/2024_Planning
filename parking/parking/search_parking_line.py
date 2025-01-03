#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from math import sqrt, atan2, degrees, pi, asin

class ParkingPathPlanner(Node):

    def __init__(self):
        super().__init__('parking_path_planner')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_closest_point = self.create_subscription(
            Point,
            '/closest_point',
            self.refer_point_callback,
            qos_profile
        )
        self.sub_marker_array = self.create_subscription(
            MarkerArray,
            '/cone_points',
            self.callback,
            qos_profile)
        
        self.pub_marker = self.create_publisher(
            Marker,
            '/max_length_marker',
            qos_profile)
        
        self.pub_goal = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile)

        self.sub_odometry = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            qos_profile)
        
        
        self.points = []
        self.current_orientation = None
        self.reference_point = None  # 시작점 또는 평행주차 시 가장 가까운 원뿔 점
        self.angle_range = (70, 110)  # 각도 범위
        self.timer = self.create_timer(1, self.timer_callback)
        self.max_length_points = None
        self.num = 0

    def refer_point_callback(self, msg):
        """가장 가까운 점 구독 콜백 함수."""
        self.reference_point = np.array([msg.x, msg.y])

    def odometry_callback(self, msg):
        """IMU 구독 콜백 함수."""
        if self.num == 0 :
            self.current_orientation = msg.pose.pose.orientation
            self.num = 1
    def callback(self, data):
        """원뿔 점 구독 콜백 함수."""
        self.points = []
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.points.append((x, y))
        self.points = np.array(self.points)

    def quaternion_to_euler(self, x, y, z, w):
        """쿼터니언을 오일러 각도로 변환."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def calculate_angle(self, point1, point2):
        """두 점 사이의 각도를 계산."""
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        return degrees(atan2(dy, dx))

    def find_points_with_angle(self, points, reference_point, angle_range, exclude_points=[]):
        """주어진 각도 범위 내의 점을 찾기."""
        selected_points = []
        for point in points:
            if np.array_equal(point, reference_point) or any(np.array_equal(point, ep) for ep in exclude_points):
                continue  # 같은 점 또는 제외된 점은 무시
            angle = self.calculate_angle(reference_point, point)
            if angle_range[0] <= angle <= angle_range[1]:
                selected_points.append(point)
        
        # 기준점(reference_point)에 가장 가까운 점 선택
        selected_points.sort(key=lambda x: sqrt((x[0] - reference_point[0])**2 + (x[1] - reference_point[1])**2))
        closest_point = selected_points[0] if selected_points else None
        
        return closest_point

    def distance(self, point1, point2):
        """두 점 사이의 거리를 계산."""
        return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def timer_callback(self):
        """주기적으로 호출되는 타이머 콜백 함수."""
        if len(self.points) == 0 or self.current_orientation is None or self.reference_point is None:
            return

        # Odometry 메시지에서 차량의 진행 방향을 계산
        roll, pitch, yaw = self.quaternion_to_euler(
            self.current_orientation.x, 
            self.current_orientation.y, 
            self.current_orientation.z, 
            self.current_orientation.w
        )
        rotate = yaw * 180 / pi
        while True: 
            if rotate < -180:
                rotate += 360

            if rotate > 180:
                rotate -= 360

            else:
                break
        
        # 진행 방향에 따라 각도 범위 업데이트
        angle_range = (rotate - 10, rotate + 10) 
        #self.get_logger().info(f"Angle range: {angle_range}")

        path = [self.reference_point.tolist()]
        current_point = self.reference_point

        while True:
            closest_point = self.find_points_with_angle(self.points, current_point, angle_range, exclude_points=path)
            
            if closest_point is None or np.array_equal(closest_point, current_point):
                break
            path.append(closest_point.tolist())
            current_point = closest_point

        max_length = 3
        too_much = 7
        max_length_points = None

        for i in range(len(path) - 1):
            dist = self.distance(path[i], path[i + 1])
            if dist > max_length and dist < too_much:
                max_length = dist
                max_length_points = (path[i], path[i + 1])
                self.max_length_points = max_length_points
        self.get_logger().info(f"Max distance: {max_length}")
        # self.get_logger().info(f"Points: {self.max_length_points}")

        # 주차 경로 생성 및 목표 지점 설정
        if self.max_length_points:
            # 목표 지점 설정
            goal_point = ((self.max_length_points[0][0]+self.max_length_points[1][0])/2 ,(self.max_length_points[0][1]+self.max_length_points[1][1])/2) #자동차 옆 거리
            self.publish_goal(goal_point)

        # 마커 발행
        if self.max_length_points:
            self.publish_marker(self.max_length_points)

    def calculate_goal_point(self, start_point, distance):
        """주차 입구에서 특정 거리만큼 떨어진 목표 지점을 계산."""
        # 현재 각도로부터 목표 지점 계산
        goal_x = start_point[0] + distance * np.cos(np.radians(self.angle_range[1]))
        goal_y = start_point[1] + distance * np.sin(np.radians(self.angle_range[1]))
        return [goal_x, goal_y]

    def publish_goal(self, goal_point):
        """목표 지점을 발행."""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_point[0]
        goal.pose.position.y = goal_point[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # 기본적인 방향 설정 (필요시 수정 가능)

        self.pub_goal.publish(goal)
        #self.get_logger().info(f"Published goal: {goal_point}")

    def publish_marker(self, points):
        """마커를 발행."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "max_length_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 선 너비
        marker.color.a = 1.0
        marker.color.g = 1.0  # 녹색

        # 마커에 점 추가
        p1 = points[0]
        p2 = points[1]

        point1 = Point()
        point1.x = p1[0]
        point1.y = p1[1]
        point1.z = 0.0

        point2 = Point()
        point2.x = p2[0]
        point2.y = p2[1]
        point2.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)

        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingPathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
