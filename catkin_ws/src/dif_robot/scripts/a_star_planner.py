#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import math
import threading
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse
from heapq import heappush, heappop
import time

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        # Map Parameters
        self.grid_size = 0.1
        self.map_width = 200
        self.map_height = 200
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin = Point(-10, -10, 0)

        # Obstacle inflation radius in cells
        # MINIMAL INFLATION FOR MAXIMUM AGGRESSION AND TIGHT PATHS. HIGH RISK OF COLLISION.
        self.inflation_radius_cells = 3 # Было 5. Ещё меньше.

        # Robot and Goal State
        self.robot_pose = Point(0, 0, 0)
        self.robot_yaw = 0
        self.goal_pose = None
        self.path = []
        self.home_pose = Point(0, 0, 0)

        # ROS Subscribers and Publishers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        
        # ROS Services
        rospy.Service('/set_goal', Empty, self.set_goal_callback)
        rospy.Service('/go_home', Empty, self.go_home_callback)

        # Control Parameters
        self.linear_speed = 0.65 # Было 0.55. Максимально высокая скорость.
        self.angular_speed = 1.8 # Было 1.5. Максимально высокая угловая скорость.
        self.rate = rospy.Rate(100) # Было 70. Исключительно высокая частота обновления для максимальной отзывчивости.

        self.last_plan_time = rospy.Time.now()

        # Stuck Detection Parameters (even more frequent check)
        self.last_robot_pose_check = Point(0,0,0)
        self.last_stuck_check_time = rospy.Time.now()
        self.stuck_threshold_distance = 0.03 # Было 0.04. Ещё меньше порог для обнаружения.
        self.stuck_check_interval = rospy.Duration(0.7) # Было 1.0. Очень частая проверка на застревание.

        # Active Obstacle Avoidance Parameters
        # EXTREMELY AGGRESSIVE. Very small buffers. High risk of collision if not perfectly tuned.
        self.critical_obstacle_distance = 0.18 # Было 0.22. Почти минимальное расстояние до экстренного стопа.
        self.safety_buffer_distance = 0.3 # Было 0.35. Локальное избегание начинается на очень близком расстоянии.
        self.is_avoiding_obstacle = False
        self.last_lidar_data = None

        # For Local Avoidance (Wall Following)
        self.obstacle_in_front = False
        self.front_ranges_idx = []
        self.left_ranges_idx = []
        self.right_ranges_idx = []

    def world_to_grid(self, x, y):
        gx = int((x - self.map_origin.x) / self.grid_size)
        gy = int((y - self.map_origin.y) / self.grid_size)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.grid_size + self.map_origin.x + self.grid_size / 2.0
        wy = gy * self.grid_size + self.map_origin.y + self.grid_size / 2.0
        return wx, wy

    def lidar_callback(self, data):
        self.last_lidar_data = data

        if not self.front_ranges_idx:
            angle_range_front = math.radians(15) # Было 18. Ещё более узкий угол для фокусировки.
            angle_range_side = math.radians(55)
            num_ranges = len(data.ranges)
            angle_per_idx = data.angle_increment

            for i in range(num_ranges):
                angle_from_front = self.normalize_angle(data.angle_min + i * angle_per_idx)
                if abs(angle_from_front) < angle_range_front:
                    self.front_ranges_idx.append(i)
                
                if math.radians(90) - angle_range_side/2 < angle_from_front < math.radians(90) + angle_range_side/2:
                    self.left_ranges_idx.append(i)
                elif math.radians(-90) - angle_range_side/2 < angle_from_front < math.radians(-90) + angle_range_side/2:
                    self.right_ranges_idx.append(i)

        min_front_range = data.range_max
        if self.front_ranges_idx:
            for idx in self.front_ranges_idx:
                r = data.ranges[idx]
                if data.range_min < r < data.range_max:
                    min_front_range = min(min_front_range, r)
        else:
             for r in data.ranges:
                if data.range_min < r < data.range_max:
                    min_front_range = min(min_front_range, r)


        if min_front_range < self.critical_obstacle_distance:
            self.obstacle_in_front = True
            if not self.is_avoiding_obstacle:
                rospy.logwarn(f"Критическое препятствие на расстоянии {min_front_range:.2f} м! Начинаю экстренный маневр.")
                self.is_avoiding_obstacle = True
                threading.Thread(target=self.emergency_avoidance).start()
        elif min_front_range < self.safety_buffer_distance:
            self.obstacle_in_front = True
        else:
            self.obstacle_in_front = False

        self.map.fill(0)
        temp_obstacles = []
        for i, r in enumerate(data.ranges):
            if data.range_min < r < data.range_max:
                angle = data.angle_min + i * data.angle_increment + self.robot_yaw
                x = self.robot_pose.x + r * math.cos(angle)
                y = self.robot_pose.y + r * math.sin(angle)
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    temp_obstacles.append((gx, gy))

        for ox, oy in temp_obstacles:
            self.map[oy][ox] = 100
            self.inflate_obstacles(ox, oy, self.inflation_radius_cells)

        self.publish_map()

    def inflate_obstacles(self, ox, oy, inflation_radius_cells):
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = ox + i, oy + j
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    # Keep inflation to 50 (partially occupied) for planning through
                    if self.map[ny][nx] == 0:
                        self.map[ny][nx] = 50

    def odom_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        if self.home_pose.x == 0 and self.home_pose.y == 0 and self.home_pose.z == 0 and \
           (self.robot_pose.x != 0 or self.robot_pose.y != 0):
            rospy.loginfo(f"Домашняя позиция установлена в: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
            self.home_pose = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid_size
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position = self.map_origin
        msg.data = self.map.flatten().tolist()
        self.map_pub.publish(msg)

    def set_goal_callback(self, req):
        rospy.loginfo("Сервис /set_goal вызван. Ожидание ввода координат цели в терминале.")
        threading.Thread(target=self._ask_user_for_goal).start()
        return EmptyResponse()

    def _ask_user_for_goal(self):
        try:
            x_str = input("Введите координату X цели (м) [от -5 до 5]: ")
            y_str = input("Введите координату Y цели (м) [от -5 до 5]: ")

            x = float(x_str)
            y = float(y_str)

            if not (self.map_origin.x <= x <= self.map_origin.x + self.map_width * self.grid_size and
                            self.map_origin.y <= y <= self.map_origin.y + self.map_height * self.grid_size):
                rospy.logerr(f"Координаты ({x:.2f}, {y:.2f}) вне допустимого диапазона карты "
                                f"[{self.map_origin.x:.2f}, {self.map_origin.x + self.map_width * self.grid_size:.2f}] "
                                f"по X и [{self.map_origin.y:.2f}, {self.map_origin.y + self.map_height * self.grid_size:.2f}] по Y.")
                return

            self.goal_pose = Point(x, y, 0)
            rospy.loginfo(f"Новая цель установлена в X={x:.2f}, Y={y:.2f} м.")
            self.plan_path()
        except ValueError:
            rospy.logerr("Неверный ввод. Пожалуйста, введите числовые значения для X и Y.")
        except Exception as e:
            rospy.logerr(f"Произошла ошибка при вводе координат: {e}")

    def go_home_callback(self, req):
        rospy.loginfo(f"Вызван сервис /go_home. Устанавливаю цель в домашнюю позицию: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f})")
        self.goal_pose = Point(self.home_pose.x, self.home_pose.y, self.home_pose.z)
        self.plan_path()
        return EmptyResponse()

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, node):
        x, y = node
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (-1, 1), (1, -1), (-1, -1)
        ]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                # Plan through partially occupied cells for maximum aggression
                if self.map[ny, nx] < 100:
                    neighbors.append((nx, ny))
        return neighbors

    def plan_path(self):
        if self.goal_pose is None:
            rospy.loginfo("Цель не установлена, планирование пути невозможно.")
            self.path = []
            return

        start = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        goal = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        if not (0 <= start[0] < self.map_width and 0 <= start[1] < self.map_height) or \
           self.map[start[1]][start[0]] == 100:
            rospy.logwarn(f"Стартовая точка ({start[0]}, {start[1]}) является препятствием или вне карты. Проверьте расположение робота и карты.")
            self.path = []
            return
        if not (0 <= goal[0] < self.map_width and 0 <= goal[1] < self.map_height) or \
           self.map[goal[1]][goal[0]] == 100:
            rospy.logwarn(f"Целевая точка ({goal[0]}, {goal[1]}) является препятствием или вне карты. Пожалуйста, выберите другую цель.")
            self.path = []
            return

        open_set = []
        heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current_f, current = heappop(open_set)

            if current == goal:
                raw_path = []
                while current in came_from:
                    raw_path.append(current)
                    current = came_from[current]
                raw_path.append(start)
                raw_path.reverse()
                self.path = self.smooth_path(raw_path)
                rospy.loginfo("Путь успешно построен.")
                self.publish_current_path()
                return

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + (math.hypot(neighbor[0] - current[0], neighbor[1] - current[1]))

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Путь не найден.")
        self.path = []

    def smooth_path(self, path):
        if len(path) < 3:
            return path
        
        smoothed_path = [path[0]]
        i = 0
        while i < len(path) - 2:
            p1 = path[i]
            p2 = path[i+1]
            p3 = path[i+2]

            wx1, wy1 = self.grid_to_world(*p1)
            wx2, wy2 = self.grid_to_world(*p2)
            wx3, wy3 = self.grid_to_world(*p3)

            area = abs((wx2 - wx1) * (wy3 - wy1) - (wx3 - wx1) * (wy2 - wy1))

            if area < 0.005: # Было 0.008. Максимально агрессивное сглаживание.
                i += 1
            else:
                smoothed_path.append(path[i+1])
                i += 1
        
        smoothed_path.append(path[-1])
        return smoothed_path

    def publish_current_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for gx, gy in self.path:
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # --- METHODS FOR ACTIVE OBSTACLE AVOIDANCE AND PATH FOLLOWING ---

    def emergency_avoidance(self):
        """
        Emergency maneuver: move back and turn if robot is too close to an obstacle.
        """
        rospy.loginfo("!!! ЭКСТРЕННЫЙ МАНЕВР ИЗБЕГАНИЯ: Опасная близость к препятствию !!!")
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.05) # Было 0.1. Практически мгновенная остановка.

        cmd.linear.x = -0.4 # Было -0.35. Максимально быстрая скорость заднего хода.
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.6) # Было 0.8. Ещё меньше времени на отъезд.

        cmd.linear.x = 0.0
        turn_direction = 1 if np.random.rand() > 0.5 else -1
        cmd.angular.z = turn_direction * self.angular_speed * 1.1 # Даже больше, чем self.angular_speed для экстренного поворота
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.7) # Было 0.9. Ещё меньше времени на поворот.

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1) # Было 0.2. Минимальная задержка после маневра.

        rospy.loginfo("Экстренный маневр завершен. Пытаюсь перепланировать.")
        self.is_avoiding_obstacle = False
        self.plan_path()

    def follow_path_and_avoid(self):
        """
        Main controller that follows the global path
        but reactively avoids obstacles if they are close.
        """
        cmd = Twist()

        if self.is_avoiding_obstacle:
            return

        if not self.path:
            self.cmd_pub.publish(cmd)
            return

        if self.goal_pose:
            goal_dist = math.hypot(self.goal_pose.x - self.robot_pose.x, self.goal_pose.y - self.robot_pose.y)
            if goal_dist < 0.04: # Было 0.05. Абсолютно минимальный порог для достижения цели.
                self.path = []
                rospy.loginfo("Цель достигнута!")
                self.cmd_pub.publish(cmd)
                self.goal_pose = None
                return

        # --- Local Avoidance / Wall Following Logic ---
        if self.obstacle_in_front:
            rospy.loginfo_throttle(0.05, "Препятствие впереди. Включаю режим объезда.") # Максимально частый лог
            if self.last_lidar_data:
                ranges = self.last_lidar_data.ranges

                front_avg_dist = self._get_avg_range(ranges, self.front_ranges_idx)
                left_avg_dist = self._get_avg_range(ranges, self.left_ranges_idx)
                right_avg_dist = self._get_avg_range(ranges, self.right_ranges_idx)

                turn_angular_z = 0.0
                
                # Move forward very aggressively if any space
                if front_avg_dist > self.critical_obstacle_distance + 0.02: # Было 0.03. Минимальный буфер для движения вперед.
                     cmd.linear.x = self.linear_speed * 0.95 # Было 0.9. Почти полная скорость во время объезда.
                else:
                    cmd.linear.x = 0.0 # Stop linear motion if directly blocked.

                # Prioritize turning towards the clearer side, use full angular speed.
                if left_avg_dist > right_avg_dist and left_avg_dist > self.safety_buffer_distance:
                    turn_angular_z = self.angular_speed # Full angular speed for swift turns.
                    rospy.loginfo_throttle(0.05, f"Объезд: Поворот влево. Left:{left_avg_dist:.2f}, Right:{right_avg_dist:.2f}")
                elif right_avg_dist > left_avg_dist and right_avg_dist > self.safety_buffer_distance:
                    turn_angular_z = -self.angular_speed # Full angular speed for swift turns.
                    rospy.loginfo_throttle(0.05, f"Объезд: Поворот вправо. Left:{left_avg_dist:.2f}, Right:{right_avg_dist:.2f}")
                else:
                    # If both directions are blocked, rotate at max speed to find opening.
                    turn_angular_z = self.angular_speed * 1.1 # Было 1.0. Ещё быстрее вращение для поиска выхода.
                    rospy.logwarn_throttle(0.05, "Объезд: Оба направления заблокированы, быстро вращаюсь.")

                cmd.angular.z = np.clip(turn_angular_z, -self.angular_speed, self.angular_speed)
                
                self.cmd_pub.publish(cmd)
                return

        # --- Global Path Following ---
        target_gx, target_gy = self.path[0]
        current_waypoint_world_x, current_waypoint_world_y = self.grid_to_world(target_gx, target_gy)
        distance_to_current_waypoint = math.hypot(current_waypoint_world_x - self.robot_pose.x,
                                                 current_waypoint_world_y - self.robot_pose.y)
        
        # If current waypoint is reached, pop it and move to the next (absolutely minimal threshold for precision)
        if distance_to_current_waypoint < 0.015: # Было 0.02. Крайне малый порог.
            if len(self.path) > 1:
                self.path.pop(0)
                rospy.loginfo(f"Переход к следующей точке пути. Осталось: {len(self.path)}")
                self.publish_current_path()
            else:
                pass

        if not self.path and self.goal_pose:
            rospy.logwarn_throttle(5, "Путь закончился, но цель не достигнута. Перепланирование...")
            self.plan_path()
            return

        target_wx, target_wy = self.grid_to_world(target_gx, target_gy)
        dx = target_wx - self.robot_pose.x
        dy = target_wy - self.robot_pose.y
        
        angle_to_waypoint = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(angle_to_waypoint - self.robot_yaw)

        # Angular speed control (EXTREMELY HIGH GAIN for maximum precision and responsiveness)
        angular_gain = 7.0 # Было 6.0. Исключительно агрессивное выравнивание.
        cmd.angular.z = np.clip(angular_gain * yaw_error, -self.angular_speed, self.angular_speed)
        
        # Linear speed control (Extremely daring with angles, but precise in slowdowns)
        if abs(yaw_error) > math.radians(55): # Было 50. Позволяет ещё больше скорости при больших углах.
            cmd.linear.x = 0.0
        else:
            linear_factor = 1.0 - abs(yaw_error) / math.radians(80) # Было 70. Позволяет ещё больше скорости при больших углах.
            linear_factor = max(0.7, linear_factor) # Было 0.6. Максимально высокая минимальная скорость.
            
            distance_factor = np.clip(distance_to_current_waypoint / 0.1, 0.7, 1.0) # Было 0.15/0.6. Минимальная дистанция замедления, высокая мин. скорость.
            
            cmd.linear.x = np.clip(self.linear_speed * linear_factor * distance_factor, 0.0, self.linear_speed)

        self.cmd_pub.publish(cmd)

    def _get_avg_range(self, ranges, indices):
        """Helper function to get average range from selected lidar indices."""
        valid_ranges = []
        for idx in indices:
            if 0 <= idx < len(ranges):
                r = ranges[idx]
                if self.last_lidar_data.range_min < r < self.last_lidar_data.range_max:
                    valid_ranges.append(r)
        if valid_ranges:
            return np.mean(valid_ranges)
        return self.last_lidar_data.range_max
        
    def normalize_angle(self, angle):
        """Normalizes an angle to the range [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def run(self):
        rospy.loginfo("Ожидание odometry для получения начальной позы робота...")
        while not rospy.is_shutdown() and \
              (self.robot_pose.x == 0 and self.robot_pose.y == 0 and self.robot_yaw == 0):
            rospy.loginfo_throttle(1, "Ожидание данных одометрии для инициализации позы робота...")
            self.rate.sleep()
        
        rospy.loginfo("Начальная поза получена. Робот готов к движению.")
        rospy.loginfo(f"Домашняя позиция робота установлена в: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f})")
        rospy.loginfo("Для установки новой цели вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /set_goal std_srvs/Empty")
        rospy.loginfo("Затем введите координаты X и Y в терминале, где запущен этот узел.")
        rospy.loginfo("Для возвращения в домашнюю позицию (где был запущен) вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /go_home std_srvs/Empty")


        while not rospy.is_shutdown():
            if self.is_avoiding_obstacle:
                self.rate.sleep()
                continue 

            # Stuck detection (very frequent check)
            if self.goal_pose and (rospy.Time.now() - self.last_stuck_check_time > self.stuck_check_interval):
                dist_moved = math.hypot(self.robot_pose.x - self.last_robot_pose_check.x,
                                         self.robot_pose.y - self.last_robot_pose_check.y)
                if dist_moved < self.stuck_threshold_distance:
                    rospy.logwarn("Робот, возможно, застрял! Запускаю экстренный маневр.")
                    self.is_avoiding_obstacle = True
                    threading.Thread(target=self.emergency_avoidance).start()
                self.last_robot_pose_check = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)
                self.last_stuck_check_time = rospy.Time.now()

            # Global path replanning (MAXIMUM ADAPTABILITY)
            if self.goal_pose and (not self.path or \
               (rospy.Time.now() - self.last_plan_time > rospy.Duration(0.5)) or \
               (not self.is_avoiding_obstacle and self.obstacle_in_front)): # Было 0.7. Теперь 0.5 сек для мгновенной реактивности.
                rospy.loginfo_throttle(0.1, "Инициирую перепланирование глобального маршрута...")
                self.plan_path()
                self.last_plan_time = rospy.Time.now()
            
            self.follow_path_and_avoid()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = AStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
