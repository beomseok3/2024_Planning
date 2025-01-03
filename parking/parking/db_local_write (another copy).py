#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


import numpy as np
import math as m
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tf_transformations import *

import sqlite3


class DBWRITE(Node):
    def __init__(self):
        super().__init__("dbwrite")
        qos_profile = QoSProfile(depth=10)
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile
        )

        self.path_x = []
        self.path_y = []
        self.path_cov = []
        self.path = []
        self.euclidean_list = []
        self.ds = 0.8
        self.i =0

    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        a = msg.pose.covariance

        cov = (
            a[0] + a[7]
        ) 

        p1 = (x, y, cov)


        if not self.euclidean_duplicate(p1):

            self.path_x.append(x)
            self.path_y.append(y)
            self.path_cov.append(cov)
        else:
            euc_dup_list = sorted(self.euclidean_list, key=lambda point: point[2])
            low_cov_point = euc_dup_list[0]
            if low_cov_point == p1:
                self.update_path_with_low_cov_point(euc_dup_list)
            self.euclidean_list.clear()  # Clear the list after handling duplicates

        if len(self.path_x) >= 3:  # 최소 3개의 포인트 필요
            self.interpolate_path()

    def euclidean_duplicate(self, p1):
        threshold = 1
        for x, y, cov in zip(self.path_x, self.path_y, self.path_cov):
            distance = m.sqrt((p1[0] - x) ** 2 + (p1[1] - y) ** 2)
            if distance <= threshold:
                self.euclidean_list.append((x, y, cov))
                return True
        return False

    def update_path_with_low_cov_point(self, euc_dup_list):
        low_cov_point = euc_dup_list[0]
        for x, y, cov in euc_dup_list[1:]:
            if x in self.path_x:
                self.path_x.remove(x)
            if y in self.path_y:
                self.path_y.remove(y)
        self.path_x.append(low_cov_point[0])
        self.path_y.append(low_cov_point[1])

    def interpolate_path(self):

        x = np.array(self.path_x)
        y = np.array(self.path_y)
        dx_ = np.diff(x)
        dy_ = np.diff(y)
        ds = np.sqrt(dx_**2+dy_**2)
        s= np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")

            self.narrow = int(s[-1]/ self.ds)

            print(self.narrow)

            s_new = np.linspace(s[0], s[-1], self.narrow)
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
            
            self.write_db()  # Save the path to the database

        except Exception as e:
            self.get_logger().error(
                f"An error occurred during spline interpolation: {e}"
            )

    def write_db(self):
        db_file = "example.db"
        try:
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            cursor.execute(
                """
            CREATE TABLE IF NOT EXISTS Path (
                idx INTEGER PRIMARY KEY,
                id TEXT NOT NULL,
                value_x REAL NOT NULL,
                value_y REAL NOT NULL,
                yaw REAL NOT NULL
            )
            """
            )
            data = [(x, y, steer) for x, y, steer in self.path]
            flag = 0
            for i,(cx, cy, cyaw) in enumerate(data):
                if 0<= i < 400:
                    id= "a1a2"
                elif 400 <= i <= 457:
                    id = "a2b1"
                elif 457 < i < 1270:
                    id ="b1b2"
                elif 1270 <= i <= 1316:
                    id = "b2c1"
                elif 1316 < id:
                     id = "c1c2"
                cursor.execute(
                    "INSERT INTO Path (idx,id, value_x, value_y, yaw) VALUES (?,?, ?, ?,?) ON CONFLICT(idx) DO UPDATE SET id=excluded.id, value_x=excluded.value_x, value_y=excluded.value_y, yaw=excluded.yaw",
                    (i,id,cx, cy, cyaw),
                )
            if self.i ==0:
                
                cursor.execute(
                    """
                CREATE TABLE IF NOT EXISTS Node (
                    start TEXT NOT NULL,
                    end TEXT NOT NULL,
                    id TEXT NOT NULL,
                    mission TEXT NOT NULL
                )
                """
                )
                _ = [("a1","a2","a1a2","driving"),("a2","b1","a2b1","avoidance")
                        ,("b1","b2","b1b2","driving"),("b2","c1","b2c1","parking"),("c1","c2","c1c2","driving")]
                for start,end,id,mission in _:
                    cursor.execute(
                        "INSERT INTO Node (start, end, id, mission) VALUES (?, ?, ?, ?)",
                        (start,end,id,mission),
                )
                self.i += 1
            conn.commit()
        except sqlite3.Error as e:
            self.get_logger().error(
                f"An error occurred during database operations: {e}"
            )
        finally:
            conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = DBWRITE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
