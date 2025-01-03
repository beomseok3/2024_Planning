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
        self.sub_domain = self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.callback_domain, qos_profile
        )
        # self.sub_local = self.create_subscription(Odometry,"localization/kinematic_state",self.callback_local,qos_profile)
        self.path_x = []
        self.path_y = []
        self.path = []

        self.distance = 0.0
        self.ds = 0.1

    def callback_domain(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if len(self.path_x) != 0:
            lx = self.path_x[-1]
            ly = self.path_y[-1]

            dist = m.sqrt((x - lx) ** 2 + (y - ly) ** 2)
            self.distance += dist

        self.path_x.append(x)
        self.path_y.append(y)

        if len(self.path_x) >= 3:  # 최소 3개의 포인트 필요
            x = np.array(self.path_x)
            y = np.array(self.path_y)
            t = np.arange(len(x))  # Use indices as parameter t

            try:
                cs_x = CubicSpline(t, x, bc_type="natural")
                cs_y = CubicSpline(t, y, bc_type="natural")

                narrow = int(self.distance / self.ds)

                print(narrow)

                t_new = np.linspace(t[0], t[-1], narrow)
                x_new = cs_x(t_new)
                y_new = cs_y(t_new)

                dx_new = cs_x(t_new, 1)
                dy_new = cs_y(t_new, 1)

                yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
                self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))

                # plt.figure()
                # plt.plot(x, y, 'o', label='data points')
                # plt.plot(x_new, y_new, '-', label='cubic spline')
                # plt.plot(t_new, yaw_new, '-r')
                # plt.legend(loc='best')
                # plt.title('Cubic Spline')
                # plt.show()

                # self.write_db()  # Save the path to the database

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
            CREATE TABLE IF NOT EXISTS data (
                id INTEGER PRIMARY KEY,
                value_x REAL NOT NULL,
                value_y REAL NOT NULL,
                steer REAL NOT NULL
            )
            """
            )
            data = [(x, y, steer) for x, y, steer in self.path]
            for i, (x, y, steer) in enumerate(data):
                cursor.execute(
                    "INSERT INTO data (id, value_x, value_y, steer) VALUES (?, ?, ?, ?) ON CONFLICT(id) DO UPDATE SET value_x=excluded.value_x, value_y=excluded.value_y, steer=excluded.steer",
                    (i, x, y, steer),
                )  # id가 같을 때만 덮어쓰기함 --> id가 느는 상황이 아니면 file명을 바꿔서 써야함

            conn.commit()
        except sqlite3.Error as e:
            self.get_logger().info(f"An error occurred during database operations: {e}")
        finally:
            conn.close()


def main(args=None):
    
    rclpy.init(args=args)
    print(args)
    node = DBWRITE()
    print(args)
    try:
        print(args)
        rclpy.spin(node)
        print(args)
    except KeyboardInterrupt:
        pass
    finally:
        print(args)
        node.destroy_node()
        print(args)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
