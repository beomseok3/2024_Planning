#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry, Path
import numpy as np
from sensor_msgs.msg import Imu
import math as m
import time as t
import sys
from tf_transformations import *
import sqlite3

from math import sqrt, atan2, degrees, pi, asin
sys.path.append("/home/ps/planning/src/parking/parking")

import reeds_shepp_planner as rs
# from utils import *


class FINDEMPTY(Node):
    def __init__(self):
        super().__init__('parking_path')
        qos_profile = QoSProfile(depth=10)

        # 현재 위치 map 기준
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )
        # 골 위치 받기 map 기준
        self.sub_goal_point = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_point_callback,
            qos_profile
        )
        # 방해물 위치 받기 map 기준
        self.sub_marker_array = self.create_subscription(
            MarkerArray,
            '/cone_points',
            self.obs_callback,
            qos_profile
        )

        self.sub_marker = self.create_subscription(
            Marker,
            '/max_length_marker',
            self.maxpoint_callback,
            qos_profile
        )

        # 차량 방향 필요할진 모르겠음
        # self.sub_odometry = self.create_subscription(
        #     Imu,
        #     '/imu/rotated',
        #     self.odometry_callback,
        #     qos_profile
        # )

        # 길 생성 map 기준?
        self.pub_path = self.create_publisher(
            Path,
            '/parking_path',
            qos_profile
        )
        self.pub_global_path = self.create_publisher(
            Path,
            '/path',
            qos_profile
        )

        self.pub_marker_array = self.create_publisher(
            MarkerArray,  # MarkerArray로 발행
            '/target_speed',
            qos_profile
        )
        self.pub_gear = self.create_publisher(
            Int32,
            "/gear",
            qos_profile
        )
        self.pub_estop = self.create_publisher(
            Int32,
            "/estop",
            qos_profile
        )
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.start = None
        self.goal = None
        self.obs = None
        self.rotate = None
        self.point1 = None
        self.point2 = None
        self.obs_margin = 2
        self.i = 0
        self.j = 0
        self.k = 0

        self.x = None
        self.y = None
        self.dx = None
        self.dy = None
        self.velocity = None
        self.gear = 2
        
        self.cx = []
        self.cy = []
        self.cyaw = []
        
        self.db =np.array([])
        db_file ="/home/ps/planning/example.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM Path")
        rows = cursor.fetchall()
        db = []
        for row in rows:
            db.append(row)
        conn.close()
        self.db =np.array(db)
        self.cx = self.db[:,2].astype(float)
        self.cy = self.db[:,3].astype(float)
        self.cyaw = self.db[:,4].astype(float)
        print(f"{self.cx},{self.cy},{self.cyaw}")
        
    def maxpoint_callback(self, data):
        self.point1 = np.array((data.points[0].x, data.points[0].y))
        self.point2 = np.array((data.points[1].x, data.points[1].y))

    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.start = np.array((x, y))
        quaternion =[msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,yaw = euler_from_quaternion(quaternion)
        rotate = yaw * 180 / pi
        while True:
            if rotate < -180:
                rotate += 360

            if rotate > 180:
                rotate -= 360

            else:
                break

        self.rotate = rotate
        if self.gear ==0:
            if  m.sqrt((self.x[0] - self.start[0])**2 + (self.y[0]-self.start[1])**2) <= 1.0 and self.k ==0 :
                msg = Int32()
                msg.data = 1
                self.pub_estop.publish(msg)
                print("{:_^57}".format("estop"))
                self.k += 1
                t.sleep(5)
                msg = Int32()
                msg.data = 0
                self.pub_estop.publish(msg)
                
                
        

    def goal_point_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.goal = np.array((x, y))
        self.parallel_parking_path()

    # def odometry_callback(self, msg):
    #     x = msg.orientation.x
    #     y = msg.orientation.y
    #     z = msg.orientation.z
    #     w = msg.orientation.w

    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw = atan2(t3, t4)

    #     rotate = yaw * 180 / pi
    #     while True:
    #         if rotate < -180:
    #             rotate += 360

    #         if rotate > 180:
    #             rotate -= 360

    #         else:
    #             break

    #     self.rotate = rotate

    def obs_callback(self, data):
        points = []
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            points.append((x, y))
        self.obs = np.array(points)

    def parallel_parking_path(self):
        if self.start is None or self.goal is None or self.point1 is None or self.point2 is None or self.i == 1:
            # self.get_logger().warn("Missing start, goal, or control points")
            return

        self.i += 1 #반복 안할거면 필요
        #t.sleep(0.5)

        rad = np.deg2rad(self.rotate - 90)
        rad2 = np.deg2rad(self.rotate + 135)

        
        offset_1 = self.obs_margin * np.sin(rad)
        offset_2 = self.obs_margin * np.cos(rad)
        offset_3 = self.obs_margin * np.sin(rad2)
        offset_4 = self.obs_margin * np.cos(rad2)

        
        PATH = [
            (self.goal[0] - offset_3, self.goal[1] + offset_4 , self.rotate),
            (self.point2[0]- offset_2 , self.point2[1]- offset_1 , self.rotate-10),
        ]
        self.path_x = []
        self.path_y = []

        for i in range(len(PATH) - 1):
            start_x = float(PATH[i][0])
            start_y = float(PATH[i][1])
            start_yaw = np.deg2rad(PATH[i][2])

            goal_x = float(PATH[i + 1][0])
            goal_y = float(PATH[i + 1][1])
            goal_yaw = np.deg2rad(PATH[i + 1][2])

            min_radius = 1  # 자동차 회전 반경
            step_size = 0.1  # 선 만들때 점의 갯수

            x, y, yaw, mode, clen = rs.reeds_shepp_planner(
                start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, min_radius, step_size
            )
            for x, y in zip(x, y):
                self.path_x.append(x)
                self.path_y.append(y)

        x = self.path_x
        y = self.path_y
        dx = np.diff(x)
        dy = np.diff(y)
        velocity = self.calculate_speed(dx, dy)

        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.velocity = velocity
        self.publish_path(x, y, dx, dy, velocity)

    def calculate_speed(self, dx, dy):
        # 현재 로봇의 방향과 각 지점 간의 방향 차이 계산
        path_yaws = np.arctan2(dy, dx) * 180 / pi
        yaw_diff = abs(self.rotate - path_yaws) % 360
        yaw_diff = np.clip(yaw_diff, 0, 180)  # 0도에서 180도 사이로 클립

        if self.rotate < 0 :

            # yaw_diff를 기반으로 속도 계산
            
            speeds_kmh = np.clip(( yaw_diff + self.rotate) / 180 *3 *-1, -5, 5)
        elif self.rotate > 0 :
            speeds_kmh = np.clip((yaw_diff - self.rotate) / 180 *3 * 1, -5, 5)
        return speeds_kmh

    def publish_path(self, x = None, y= None, dx= None, dy= None, velocity= None):
        
        if x is not None :
            path = Path()
            path.header.frame_id = 'map'

            for i in range(len(x) - 1):
                pose = PoseStamped()
                pose.pose.position.x = x[i]
                pose.pose.position.y = y[i]
                pose.pose.position.z = -1.0
                yaw = np.arctan2(dy[i], dx[i])
                q = self.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                path.poses.append(pose)

            self.pub_path.publish(path)

            marker_array = MarkerArray()
            for i in range(len(x) - 1):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.ns = "velocity"
                marker.id = i
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.pose.position.x = x[i]
                marker.pose.position.y = y[i]
                marker.pose.position.z = -0.5
                marker.pose.orientation.w = 1.0

                if i < len(velocity):
                    marker.text = str(round(velocity[i], 2))

                marker_array.markers.append(marker)
            self.pub_marker_array.publish(marker_array)
    
    def gear_publisher(self,gear):
        msg = Int32()
        msg. data = gear
        self.gear = gear
        self.pub_gear.publish(msg)
        self.get_logger().info(f"\ngear = {gear}\n")
        
    

    def timer_callback(self):
        if self.x == None:
                path = Path()
                path.header.frame_id = 'map'

                for i in range(len(self.cx) - 1):
                    pose = PoseStamped()
                    pose.pose.position.x = self.cx[i]
                    pose.pose.position.y = self.cy[i]
                    pose.pose.position.z = -1.0
                    q = self.quaternion_from_euler(0, 0, self.cyaw[i])
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    path.poses.append(pose)

                self.pub_global_path.publish(path)
        else: 
            a = (self.start[0] -self.x[-1])**2 + (self.start[1] -self.y[-1])**2  > 1
            if a and self.j == 0:
                path = Path()
                path.header.frame_id = 'map'

                for i in range(len(self.cx) - 1):
                    pose = PoseStamped()
                    pose.pose.position.x = self.cx[i]
                    pose.pose.position.y = self.cy[i]
                    pose.pose.position.z = -1.0
                    q = self.quaternion_from_euler(0, 0, self.cyaw[i])
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    path.poses.append(pose)

                self.pub_global_path.publish(path)
            else:
                if self.k ==0:
                    self.gear_publisher(0)
                else:
                    self.gear_publisher(2)
                    
                self.publish_path(self.x,self.y, self.dx, self.dy, self.velocity)
                self.j = 1

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]
    


def main(args=None):
    rclpy.init(args=args)
    node = FINDEMPTY()
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
