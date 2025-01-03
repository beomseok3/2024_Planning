#라인 변경 알고리즘을 위한 테스트

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from scipy.cluster.hierarchy import linkage, fcluster
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from math import *

#라인 만드는 코드
from scipy.interpolate import CubicSpline



class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        # 내 위치
        self.sub_odm = self.create_subscription(Odometry, '/localization/kinematic_state', self.call_odom, 10)
        #장애물 위치 #저장하는 방식이 나을듯함
        self.sub_obs = self.create_subscription(PointCloud2, '/cloud_filtered',  self.call_obs, 10)


        #글로벌 패스
        self.pub_global1 = self.create_publisher(Path, '/global_path1', 10) #나중에 map에서 받아오겠지?
        self.pub_global2 = self.create_publisher(Path, '/global_path2', 10)
        #예상 패스들
        self.pub_maybe_path1 = self.create_publisher(Path, '/path1', 10)
        self.pub_maybe_path2 = self.create_publisher(Path, '/path2', 10)
        self.pub_maybe_path3 = self.create_publisher(Path, '/path3', 10)
        self.pub_maybe_path4 = self.create_publisher(Path, '/path4', 10)
        self.pub_maybe_path5 = self.create_publisher(Path, '/path5', 10)
        self.pub_maybe_path6 = self.create_publisher(Path, '/path6', 10)
        self.pub_maybe_path7 = self.create_publisher(Path, '/path7', 10)
        self.pub_maybe_path8 = self.create_publisher(Path, '/path8', 10) 
        self.pub_maybe_path9 = self.create_publisher(Path, '/path9', 10) 



        self.timer = self.create_timer(0.1, self.timer_callback)

        
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None

        self.ori_x = None
        self.ori_y = None
        self.ori_z = None
        self.ori_w = None

        self.odom_pose = np.array([0.0, 0.0])
        self.odom_orientation = [0.0, 0.0, 0.0, 1.0]  # Default quaternion (no rotation)

        self.global_path_points1 = None #내가 원하는 경로
        self.global_path_points2 = None
        self.g_path = None

        self.obs = None
        self.avoid_path = None

        self.obstacle = None
        self.once = 0
        self.tochange = None 

        self.togo_path = None
        self.current_path = None

        self.num = 0
        
        self.local_points = None #이걸로 끝내야됨 제발
        self.maybe_path = None #여기다가 추가
        

    def call_odom(self, msg):

        #내 위치 저장
        
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.pose_z = msg.pose.pose.position.z

        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w

        self.odom_pose = np.array([self.pose_x, self.pose_y])
        self.odom_orientation = [
            self.ori_x,
            self.ori_y,
            self.ori_z,
            self.ori_w
        ]
        

    def call_obs(self, msg):

        #포인트 크라우드 데이터 받아오기
        obs = list(pc2.read_points(msg, field_names=("x", "y"), skip_nans=True))
        
        if not obs:
            self.obs = []
            return
        
        if not obs:
            return
        
        #float 현태로 바꾸기
        coords = np.array([(p[0], p[1]) for p in obs], dtype=np.float64)
        
        #클러스터
        distance_threshold = 0.5
        Z = linkage(coords, method='average')
        clusters = fcluster(Z, distance_threshold, criterion='distance')
        
        unique_clusters = np.unique(clusters)
        new_cluster_centers = []
        for cluster_id in unique_clusters:
            cluster_obs = coords[clusters == cluster_id]
            cluster_center = np.mean(cluster_obs, axis=0)
            new_cluster_centers.append(cluster_center)
        
        new_cluster_centers = np.array(new_cluster_centers)
        transformed_centers = self.transform_cluster_centers(new_cluster_centers)
        
        self.obs = np.array(transformed_centers)
        
    def rotate_points(self, points, angle):
        angle_radians = np.deg2rad(angle)
        rotation_matrix = np.array(
            [
                [np.cos(angle_radians), -np.sin(angle_radians)],
                [np.sin(angle_radians), np.cos(angle_radians)],
            ]
        )
        rotated_points = np.dot(points, rotation_matrix)
        return rotated_points

    def transform_cluster_centers(self, cluster_centers):
        if len(cluster_centers) == 0:
            return np.array([])
        euler = euler_from_quaternion(self.odom_orientation)
        _, _, yaw = euler

        rotation_angle = np.rad2deg(-yaw)

        rotated_centers = self.rotate_points(cluster_centers, rotation_angle)

        transformed_centers = rotated_centers + self.odom_pose

        return transformed_centers    

        
    


    def publish_global_path(self, wx,wy, num=None):
        
        # Interpolate y values given x using CubicSpline
        cs_x = CubicSpline(range(len(wx)), wx)
        cs_y = CubicSpline(range(len(wy)), wy)
        
        # Sampling intervals for generating the path
        s = np.linspace(0, len(wx) - 1, num=500)
        
        # Interpolated path coordinates
        rx = cs_x(s)
        ry = cs_y(s)

        
        # Save path and direction data
        path_points = np.vstack((rx, ry)).T


        if num == 1:
            if self.global_path_points1 is not None :
                path_points = self.global_path_points1
        if num == 2:
            if self.global_path_points2 is not None :
                path_points = self.global_path_points2

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Compute orientation from current position
            yaw = np.arctan2(y - self.pose_y, x - self.pose_x)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)
        if num == 1 :
                
            # Publish the global path
            self.pub_global1.publish(path)
            
            # Save global path for later use
            self.global_path_points1 = path_points
        
        if num == 2 :

            # Publish the global path
            self.pub_global2.publish(path)
            
            # Save global path for later use
            self.global_path_points2 = path_points

    
    def is_path_near_obstacle(self, path, threshold=0.3): #장애물이 있으면 True, path의 index값 줌
        if path is None:
            return False, None
        
        for i, point in enumerate(path):
            if self.is_point_near_obstacle(point, threshold):
                return True , i
        return False, None

    def is_point_near_obstacle(self, point, threshold):
        point = np.array(point)
        for obs in self.obs:
            obs = np.array(obs)
            distance = np.linalg.norm(point - obs)
            if distance < threshold:
                return True
        return False
    
    def find_closest_point_index(self,point = None):
        # 내 위치 상에서 글로벌 패스 위치 (인덱스)
        if point is None :
            point = self.odom_pose
        index1 = np.argmin(np.linalg.norm(np.array(self.global_path_points1) - point, axis=1))
        index2 = np.argmin(np.linalg.norm(np.array(self.global_path_points2) - point, axis=1))
        

        distance1 = dist(np.array(self.global_path_points1[index1]) ,point)
        distance2 = dist(np.array(self.global_path_points2[index2]) ,point)

        if distance1 < distance2 :
            self.num = 1
        else: 
            self.num = 2
        return index1 , index2, self.num
        
         
    def check_obstacle(self, current_path = None):
        global wx
        global wy 
        #글로벌 패스 경로에 대한 장애물 위치
        
        if current_path is not None :
            self.current_path = current_path#사실 필요한건지 모르겠음
        if self.current_path is None:
            self.current_path = self.g_path
            self.local_points = self.g_path.tolist()#처음 로컬 패스를 글로벌로 지정
        self.obstacle, index =self.is_path_near_obstacle(self.current_path) #글로벌 패스에 장애물 위치  #나중에 수정
        
        


        if self.obstacle : #장애물이 있으면?
            #그 인덱스만 index2로 이동
            
            index1, index2,num = self.find_closest_point_index(list(self.current_path[index]))
            print(index2,num)


            
            if self.num ==1:
          
                wx = list(self.current_path[:index-20,0])+list(self.global_path_points2[index2:,0])
            
                wy = list(self.current_path[:index-20,1])+list(self.global_path_points2[index2:,1])
                
                
                p0 = self.current_path[index-90]
                p1 = self.current_path[index-20]
                p2 = self.global_path_points2[index2-30]
                p3 = self.global_path_points2[index2+30]
                b_x,b_y = self.quad_bez(p0,p1,p2,p3)
                 
                self.local_points = list(self.local_points[:index-90])
                
                for x,y in zip(b_x,b_y):
                    self.local_points.append((x,y))

                for point in self.global_path_points2[index2+30:] :
                    self.local_points.append(point)
                
                
                dx = np.diff(b_x)
                dy = np.diff(b_y)

                self.publish_bevier(b_x, b_y, dx, dy,2)

               
                
            elif num ==2:
                wx = list(self.current_path[:index-20,0])+list(self.global_path_points1[index1:,0])
            
                wy = list(self.current_path[:index-20,1])+list(self.global_path_points1[index1:,1])


                p0 = self.current_path[index-80]
                p1 = self.current_path[index-10]
                p2 = self.global_path_points1[index1-20]
                p3 = self.global_path_points1[index1+40]
                b_x,b_y = self.quad_bez(p0,p1,p2,p3)

                self.local_points = list(self.local_points[:index-80])
                for x,y in zip(b_x,b_y):
                    self.local_points.append((x,y))
                    
                for point in self.global_path_points1[index1+40:] :
                    self.local_points.append(point)
          
               

             
                
                dx = np.diff(b_x)
                dy = np.diff(b_y)

                self.publish_bevier(b_x, b_y, dx, dy,3)


                
            self.current_path = self.publish_maybe_path(wx,wy) #path 만듦
            
        

        if not self.obstacle:    
            pass
            # self.current_path = self.publish_maybe_path(wx,wy)
            


            
            wx = self.current_path[:,0]
            wy = self.current_path[:,1]

            self.publish_maybe_path(wx,wy,6)


            
            
        


        
        

    def global_path(self):
        index1 ,_,line_num = self.find_closest_point_index()

        global_path = np.array(self.global_path_points1[index1:])

        self.g_path = global_path
        self.publish_maybe_path(global_path[:,0],global_path[:,1],9)
        
        

    def making_wp(self):
        ##num1 직진
        wx = [ i *1 for i in range(9)] 

        wy = [  i *0 for i in range(9)] 
      


        

        array = [[x,y] for x,y in zip(wx,wy)]

        array = self.transform_cluster_centers(array)
        wx =array[:,0]
        wy = array[:,1]
        self.publish_maybe_path(wx,wy,1)


        ##num2 바로 왼쪽
        radius = 2  #도로와 도로간격
        wx = [0,1]+[1 +radius * sin(radians(theta)) for theta in range (0,90,15)] + [1+radius*2 - radius * cos(radians(theta)) for theta in range (0,90,15)]
        
        wy = [0,0]+[radius - radius * cos(radians(theta)) for theta in range (0,90,15)] + [ radius + radius * sin(radians(theta)) for theta in range (0,90,15)]
        
        
        array = [[x,y] for x,y in zip(wx,wy)]

        array = self.transform_cluster_centers(array)
        wx =array[:,0]
        wy = array[:,1]

        self.publish_maybe_path(wx,wy,2)


        ##num3 직진 왼쪽
        radius = 1.6
        wx = [0,1,2,3,4]+[4 +radius * sin(radians(theta)) for theta in range (0,90,15)] + [4+radius*2 - radius * cos(radians(theta)) for theta in range (0,90,15)]
        
        wy = [0,0,0,0,0]+[radius - radius * cos(radians(theta)) for theta in range (0,90,15)] + [ radius + radius * sin(radians(theta)) for theta in range (0,90,15)]
        
        array = [[x,y] for x,y in zip(wx,wy)]

        array = self.transform_cluster_centers(array)
        wx =array[:,0]
        wy = array[:,1]

        self.publish_maybe_path(wx,wy,3)

        ##num3 바로 오른쪽
        radius = 1.6
        wx = [0,1]+[1 +radius * sin(radians(theta)) for theta in range (0,90,15)] + [1+radius*2 - radius * cos(radians(theta)) for theta in range (0,90,15)]
        
        wy = [0,0]+[-radius + radius * cos(radians(theta)) for theta in range (0,90,15)] + [ -radius - radius * sin(radians(theta)) for theta in range (0,90,15)]
        
        array = [[x,y] for x,y in zip(wx,wy)]

        array = self.transform_cluster_centers(array)
        wx =array[:,0]
        wy = array[:,1]

        self.publish_maybe_path(wx,wy,4)

        ##num3 직진 오른쪽
        radius = 2
        wx = [0,1,2,3,4]+[4 +radius * sin(radians(theta)) for theta in range (0,90,15)] + [4+radius*2 - radius * cos(radians(theta)) for theta in range (0,90,15)]
        
        wy = [0,0,0,0,0]+[- radius + radius * cos(radians(theta)) for theta in range (0,90,15)] + [ -radius - radius * sin(radians(theta)) for theta in range (0,90,15)]
        
        array = [[x,y] for x,y in zip(wx,wy)]

        array = self.transform_cluster_centers(array)
        wx =array[:,0]
        wy = array[:,1]

        self.publish_maybe_path(wx,wy,5)
    
        ##num4 장애물 회피
        if self.obstacle :
            
            
            
            
            pass

        
    def publish_maybe_path(self, wx,wy, num = None): #cubicspline
    
        wx = wx
        wy = wy
        
        # Interpolate y values given x using CubicSpline
        cs_x = CubicSpline(range(len(wx)), wx)
        cs_y = CubicSpline(range(len(wy)), wy)
        
        # Sampling intervals for generating the path
        s = np.linspace(0, len(wx) - 1, num=500)
        
        # Interpolated path coordinates
        rx = cs_x(s)
        ry = cs_y(s)

        
        # Save path and direction data
        path_points = np.vstack((rx, ry)).T




        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Compute orientation from current position
            yaw = np.arctan2(y - self.pose_y, x - self.pose_x)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.poses.append(pose)
        if num == 1: 
            self.pub_maybe_path1.publish(path)
        if num == 2: 
            self.pub_maybe_path2.publish(path)
        if num == 3: 
            self.pub_maybe_path3.publish(path)
        if num == 4:
            self.pub_maybe_path4.publish(path)
        if num == 5:
            self.pub_maybe_path5.publish(path)
        if num == 6:
            self.pub_maybe_path6.publish(path)
        if num == 7:
            self.pub_maybe_path7.publish(path)
        if num == 8:
            self.pub_maybe_path8.publish(path)
        if num == 9:
            self.pub_maybe_path9.publish(path)
            
        return path_points
    
    
    def quad_bez(self, p0, p1, p2, p3, t=100):
        t1 = np.linspace(0, 1, t)
    
        b_x = ((1 - t1) ** 3 * p0[0] +
                3 * (1 - t1) ** 2 * t1 * p1[0] +
                3 * (1 - t1) ** 1 * t1 ** 2 * p2[0] +
                t1 ** 3 * p3[0])
        b_y = ((1 - t1) ** 3 * p0[1] +
                3 * (1 - t1) ** 2 * t1 * p1[1] +
                3 * (1 - t1) ** 1 * t1 ** 2 * p2[1] +
                t1 ** 3 * p3[1])
        return b_x, b_y
        
    def publish_bevier(self, mid_bezier_x, mid_bezier_y, dx, dy,num):
        path = Path()
        path.header.frame_id = 'map'

        for x, y, dx, dy in zip(mid_bezier_x, mid_bezier_y, dx, dy):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0
            
            yaw = np.arctan2(dy, dx)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        if num == 2:
            self.pub_maybe_path2.publish(path)
        if num == 3:
            self.pub_maybe_path3.publish(path)
        

    def publish_local_path(self,points):
        # print(points)
        # print(type(points))
        x = []
        y = []

        for x_points,y_points, in points:
            x.append(x_points)
            y.append(y_points)
        dx = np.diff(x)
        dy = np.diff(y)

        path_points = []

        path = Path()
        path.header.frame_id = 'map'

        for x, y, dx, dy in zip(x,y ,dx, dy):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -1.0
            
            path_points.append((x,y))

            yaw = np.arctan2(dy, dx)
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(pose)

        
        self.pub_maybe_path1.publish(path)
        # self.local_points = path_points
       




    def timer_callback(self):
        if self.pose_x is not None and self.obs is not None:
            self.publish_global_path(wx = [16343.8,16390.6],
                wy = [28528.5,28528.5],num=1)  #1차선 center line
            
            self.publish_global_path(wx = [16343.3,16390.6],
                wy = [28525.4,28525.4],num=2)    #2차선 center line
            
            #self.making_wp()
            
            self.global_path() #나중엔 받아오지 않을까?

            self.check_obstacle() #내 길에 obstacle 확인

            
            self.publish_local_path(self.local_points)
            
            # print("장애물",self.obstacle)
            # print("points", len(self.local_points))
            # 
            # print(self.local_points)
            
            if self.obstacle :
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()