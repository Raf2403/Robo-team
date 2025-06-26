#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from a_star_planner import a_star

class RobotNavigator:
    def init(self):
        rospy.init_node('robot_navigator', anonymous=True)
        self.grid = None
        self.grid_width = 210  # 21 м / 0.1 м = 210 клеток по X
        self.grid_height = 150  # 15 м / 0.1 м = 150 клеток по Y
        self.grid_resolution = 0.1  # Разрешение карты, м/клетку
        self.map_origin = (-13.0, -8.0)  # Начало мира (минимальные X, Y стен)
        self.robot_pose = (0, 0)  # Позиция робота (x, y) в метрах
        self.goal = (7.4, 4.6)  # Цель в метрах (около Wall_13)
        self.path = None
        
        # Подписки и публикации
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        
        # Таймер для обновления траектории
        rospy.Timer(rospy.Duration(0.5), self.update_trajectory)

    def map_callback(self, data):
        """Получение карты"""
        self.grid = list(data.data)
        self.grid_width = data.info.width
        self.grid_height = data.info.height
        self.grid_resolution = data.info.resolution
        self.map_origin = (data.info.origin.position.x, data.info.origin.position.y)
        rospy.loginfo("Карта получена: %d x %d", self.grid_width, self.grid_height)
        self.update_path()

    def odom_callback(self, data):
        """Обновление позиции робота"""
        self.robot_pose = (data.pose.pose.position.x, data.pose.pose.position.y)

    def lidar_callback(self, data):
        """Обработка данных лидара"""
        obstacles = []
        for i, distance in enumerate(data.ranges):
            if data.range_min < distance < data.range_max:
                angle = data.angle_min + i * data.angle_increment
                x = self.robot_pose[0] + distance * np.cos(angle)
                y = self.robot_pose[1] + distance * np.sin(angle)
                grid_x = int((x - self.map_origin[0]) / self.grid_resolution)
                grid_y = int((y - self.map_origin[1]) / self.grid_resolution)
                if 0 <= grid_x < self.grid_height and 0 <= grid_y < self.grid_width:
                    idx = grid_x * self.grid_width + grid_y
                    self.grid[idx] = 100  # Отметить препятствие
                    obstacles.append((grid_x, grid_y))
        if obstacles and self.path:
            for obs in obstacles:
                if obs in self.path:
                    rospy.loginfo("Препятствие на пути, перепланирование...")
                    self.update_path()
                    break

    def update_path(self):
        """Перепланирование пути"""
        if self.grid is None:
            return
        start = (int((self.robot_pose[0] - self.map_origin[0]) / self.grid_resolution),
                 int((self.robot_pose[1] - self.map_origin[1]) / self.grid_resolution))
        goal = (int((self.goal[0] - self.map_origin[0]) / self.grid_resolution),
                int((self.goal[1] - self.map_origin[1]) / self.grid_resolution))
        self.path = a_star(self.grid, start, goal, self.grid_width, self.grid_height)
        if self.path:
            rospy.loginfo("Новый путь: %s", self.path)
            self.publish_path()
        else:
            rospy.logwarn("Путь не найден!")

    def publish_path(self):
        """Публикация пути для RViz"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for point in self.path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0] * self.grid_resolution + self.map_origin[0]
            pose.pose.position.y = point[1] * self.grid_resolution + self.map_origin[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def update_trajectory(self, event):
        """Следование по пути"""
        if not self.path or len(self.path) < 2:
            return
        next_point = self.path[1]
        target_x = next_point[0] * self.grid_resolution + self.map_origin[0]
        target_y = next_point[1] * self.grid_resolution + self.map_origin[1]
        
        twist = Twist()
        dx = target_x - self.robot_pose[0]
        dy = target_y - self.robot_pose[1]
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        
        if distance > 0.1:  # Если не достигли точки
            twist.linear.x = 0.2  # Настройте скорость
            twist.angular.z = 0.5 * angle
        else:
            self.path.pop(0)  # Удаляем достигнутую точку
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if name == 'main':
    try:
        navigator = RobotNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass