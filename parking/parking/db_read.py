#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt

import sqlite3

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler


class DB_READER(Node):
    def __init__(self):
        super().__init__("db_reader")
        qos_profile = QoSProfile(depth=10)
        self.pub_path = self.create_publisher(Path, "path", qos_profile)
        self.db = []
        self.x = []
        self.y = []
        self.steer = []
        self.db_read()

    def db_read(self):
        db_file = "example.db"
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM data")
        rows = cursor.fetchall()
        print(rows)
        for row in rows:
            self.db.append(row)
        conn.close()
        self.publish_path()

    def publish_path(self):
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for id,x, y, yaw in self.db:
            # if id == "b2c1":
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                quaternion = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path.poses.append(pose)
            # else:
            #     continue
            
        self.pub_path.publish(path)




def main(args=None):
    rclpy.init(args=args)
    node = DB_READER()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
