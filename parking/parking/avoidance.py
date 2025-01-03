import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math as m

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray,Marker
from std_msgs.msg import Header, String
from tf_transformations import *

from nav_msgs.msg import Path, Odometry

from scipy.interpolate import CubicSpline

class AVOIDANCE(Node):
    def __init__(self):
        
        super().__init__("avoidance")
        qos_profile = QoSProfile(depth=10)
        
        self.sub_box = self.create_subscription(
            MarkerArray,"markers",self.callback_box,qos_profile
        )
        
        self.pub_marker = self.create_publisher(
            MarkerArray, "domain_origin" , qos_profile
        )
        
        self.pub_path = self.create_publisher(
            Path, "path", qos_profile
        )
        
        
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.path_tf, qos_profile
        )
        self.pub_parking_flag = self.create_publisher(String,"parking_flag",qos_profile)
        
        #detection
        self.max_x = 0
        self.min_x = 0
        self.max_y = 0
        self.min_y = 0
        self.orientatioin = 0.0
        
        #path_plan
        self.domain = []
        self.path_x = np.array([])
        self.path_y = np.array([])
        self.path = []
        self.j = 0
        self.k = 0
        self.local = ()
        self.minus_path = []
        self.plus_path = []
        self.flag = 0
        
        
        
           
    def callback_box(self,msg):
        for marker in msg.markers:
            max_x = marker.points[0].x
            max_y = marker.points[0].y
            min_x = marker.points[1].x
            min_y = marker.points[3].y
            center = ((max_x + min_x)/2,(max_y + min_y)/2)
            length = m.sqrt(center[0]**2 + center[1]**2)
            width = abs(max_y-min_y)
            if width > 0.9 and length < 25:
                self.max_x = marker.points[0].x
                self.max_y = marker.points[0].y
                self.min_x = marker.points[1].x
                self.min_y = marker.points[3].y
                width = abs(self.max_y-self.min_y)
                self.get_logger().info(f"detect:{self.min_y},{self.max_y},{width},{length}")
                self.flag = 1
                self.publish_markers_()
    
    
            
    def path_tf(self, msg):
        if self.flag == 1:
            translation = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            rotation = [0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            _, _, yaw = euler_from_quaternion(rotation)
            if  self. j< 1:
                self.orientatioin = yaw
                self.minus_tf(translation,yaw)
                print("1")
                self.publish_path()
            else:
                if self. j == 1:
                    if m.sqrt((self.minus_path[-1][0] - translation[0])**2 + (self.minus_path[-1][1] - translation[1])**2) < 0.5:
                        self.get_logger().info("done")
                        self.j += 1
                        self.path = np.array([])
                        self.minus_path.clear()
                        self.domain.clear()
                        return
                    print("2")
                    self. publish_path()
                elif self.k < 1 and self.j == 2 : #and (yaw - self.orientatioin) < 5 굳이 필요 없어 보임
                    self.plus_tf(translation,yaw)
                elif self.k ==1:
                    print("3")
                    self.publish_path()
                    if m.sqrt((translation[0]-self.plus_path[-1][0])**2 + (translation[1]-self.plus_path[-1][1])**2) < 1.0:
                        msg = String()
                        msg.data = "end"
                        self.pub_parking_flag.publish(msg)
        else:
            pass
                        
    def minus_tf(self,translation,yaw):
        self.get_logger().info("minus_path")
        '''
                (3.0,0),(4,-0.25),(5,-0.75),
                (7.0,-1.0),(8.0,-1.25)
                '''
    
        self.domain.extend(
            [ (0,0),(2,-0.25),(2.25,-0.5),
            (self.min_x -2,self. min_y -2),
                (self.min_x,self.min_y - 2),
                (self.min_x + 1.0,self.min_y -2.0),
                (self.min_x + 2.0,self.min_y -2.0),
                (self.min_x + 3.0,self.min_y -2.0),
                
                
            ]
        
        )
        points = np.array(self.domain)
        self.path_x = points[:, 0]
        self.path_y = points[:, 1]
        self.interpolate_path()
        
        self.path = np.array(self.path)
        self.path[:, 2] += yaw
        
                # 회전 행렬을 생성하여 적용
        rotation_matrix = np.array([
            [np.cos(-yaw), np.sin(-yaw)],
            [-np.sin(-yaw), np.cos(-yaw)]
        ])
        # 좌표 (x, y)만 추출하여 회전 적용
        self.path[:, :2] = np.dot(self.path[:, :2], rotation_matrix.T)

        # Translation 적용 (벡터화된 연산 사용)
        self.path[:, 0] += translation[0]
        self.path[:, 1] += translation[1]
        
        self.minus_path = self.path.tolist()
        self.j += 1
        
        
    def plus_tf(self,translation,yaw):
        self.get_logger().info("plus_path")
        self.domain.extend(
            [ (0,0),
                (self.min_x-0.1,self.max_y -0.2),
                (self.min_x,self.max_y + 1.75),
                (self.min_x + 1,self.max_y + 1.75),
            ]
        
        )
        points = np.array(self.domain)
        self.path_x = points[:, 0]
        self.path_y = points[:, 1]
        self.interpolate_path()
        
        
        self.path = np.array(self.path)
        self.path[:, 2] += yaw
        
        # 회전 행렬을 생성하여 적용
        rotation_matrix = np.array([
            [np.cos(-yaw), np.sin(-yaw)],
            [-np.sin(-yaw), np.cos(-yaw)]
        ])

        # 좌표 (x, y)만 추출하여 회전 적용
        self.path[:, :2] = np.dot(self.path[:, :2], rotation_matrix.T)

        # Translation 적용 (벡터화된 연산 사용)
        self.path[:,0] += translation[0]
        self.path[:,1] += translation[1]

        self.plus_path = self.path.tolist()
        self.k += 1
        


        
    def interpolate_path(self):
        x = np.array(self.path_x)
        y = np.array(self.path_y)  
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx**2 + dy**2)
        s = np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")

            s_new = np.linspace(s[0], s[-1], 90)
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]
            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
        
        except Exception as e:
            self.get_logger().error(
                f"An error occurred during spline interpolation: {e}"
            )
            
            
    def publish_path(self):
        if self.k == 0:
            path = Path()
            path.header = Header()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"
            for x, y, steer in self.minus_path:
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
        else:
            path = Path()
            path.header = Header()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"
            for x, y, steer in self.plus_path:
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
            
    
    def publish_markers_(self):
        marker_array = MarkerArray()

        area = [
            (x, y) for x in [self.min_x, self.max_x] for y in [self.min_y, self.max_y]
        ]

        for i, (x, y) in enumerate(area):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "area"
            marker.id =  i 
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x  # 각 마커를 서로 다른 위치에 배치
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rclpy.duration.Duration(
                seconds=0
            ).to_msg()  # 무한 수명 설정

            marker_array.markers.append(marker)

        self.pub_marker.publish(marker_array)
            
def main(args=None):
    rclpy.init(args=args)
    node = AVOIDANCE()
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