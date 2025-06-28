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

class DStarLitePlanner:
    def __init__(self):
        rospy.init_node('d_star_lite_planner')

        # Map Parameters
        self.grid_size = 0.1 # meters per cell
        self.map_width = 200 # cells
        self.map_height = 200 # cells
        # Initialize map with unknown cells (value -1) or free (0)
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8) 
        self.map_origin = Point(-10, -10, 0) # Bottom-left corner of the map in world coords

        # Inflation radius for obstacles (in cells)
        self.inflation_radius_cells = 4 # Moderate inflation for safety

        # Robot and Goal State
        self.robot_pose = Point(0, 0, 0)
        self.robot_yaw = 0
        self.goal_pose = None
        self.path = []
        self.home_pose = Point(0, 0, 0)

        # D* Lite Specific Variables
        self.g = {} # Cost from start to node
        self.rhs = {} # Heuristic cost from start to node (g_prime)
        self.U = [] # Priority Queue
        self.k_m = 0 # Cost change accumulation
        
        self.last_start_node = None # For tracking changes in start node for k_m

        # ROS Subscribers and Publishers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/d_star_lite_path", Path, queue_size=1)
        
        # ROS Services
        rospy.Service('/set_goal_dstar', Empty, self.set_goal_callback)
        rospy.Service('/go_home_dstar', Empty, self.go_home_callback)

        # Control Parameters
        self.linear_speed = 0.5 # m/s
        self.angular_speed = 1.2 # rad/s
        self.rate = rospy.Rate(50) # Hz

        self.last_plan_time = rospy.Time.now()
        self.replan_interval = rospy.Duration(0.7) # Replan every 0.7 seconds

        # Obstacle Avoidance (simple stop-and-go for D* Lite integration)
        self.critical_obstacle_distance = 0.3 # m
        self.obstacle_detected_front = False

    def world_to_grid(self, x, y):
        gx = int((x - self.map_origin.x) / self.grid_size)
        gy = int((y - self.map_origin.y) / self.grid_size)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.grid_size + self.map_origin.x + self.grid_size / 2.0
        wy = gy * self.grid_size + self.map_origin.y + self.grid_size / 2.0
        return wx, wy

    def lidar_callback(self, data):
        # Create a temporary map to detect changes
        new_obstacles_map = np.zeros_like(self.map)
        
        min_front_range = data.range_max
        front_angle_threshold = math.radians(20) # +/- 20 degrees in front

        for i, r in enumerate(data.ranges):
            if data.range_min < r < data.range_max:
                angle_global = data.angle_min + i * data.angle_increment + self.robot_yaw
                x = self.robot_pose.x + r * math.cos(angle_global)
                y = self.robot_pose.y + r * math.sin(angle_global)
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    new_obstacles_map[gy][gx] = 100 # Mark as occupied

                # Check for critical obstacle in front
                if abs(self.normalize_angle(data.angle_min + i * data.angle_increment)) < front_angle_threshold:
                    min_front_range = min(min_front_range, r)

        self.obstacle_detected_front = min_front_range < self.critical_obstacle_distance

        # Detect changes and update D* Lite
        changes_detected = False
        temp_map = np.copy(self.map) # Current map state
        for y in range(self.map_height):
            for x in range(self.map_width):
                current_val = temp_map[y][x]
                new_val = new_obstacles_map[y][x]
                
                # Apply inflation to new_obstacles_map for planning
                if new_val == 100:
                    self.inflate_obstacles(x, y, self.inflation_radius_cells, new_obstacles_map)

                # Check if cell status changed (occupied or became free)
                if current_val != new_obstacles_map[y][x]:
                    changes_detected = True
                    self.map[y][x] = new_obstacles_map[y][x] # Update internal map
                    # Inform D* Lite about the change
                    self.update_node((x, y))
        
        # Publish the map even if no changes for visualization
        self.publish_map()
        
        if changes_detected and self.goal_pose:
            rospy.loginfo("Изменения на карте обнаружены! Перепланирую с D* Lite.")
            self.compute_shortest_path()


    def inflate_obstacles(self, ox, oy, inflation_radius_cells, target_map):
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = ox + i, oy + j
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    # Mark inflated cells as partially occupied (50) for planning around
                    if target_map[ny][nx] == 0: # Only if not already a hard obstacle
                        target_map[ny][nx] = 50


    def odom_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        if self.home_pose.x == 0 and self.home_pose.y == 0 and self.home_pose.z == 0 and \
           (self.robot_pose.x != 0 or self.robot_pose.y != 0):
            rospy.loginfo(f"Домашняя позиция для D* Lite установлена в: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
            self.home_pose = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)
            self.last_start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)

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
        rospy.loginfo("Сервис /set_goal_dstar вызван. Ожидание ввода координат цели в терминале.")
        threading.Thread(target=self._ask_user_for_goal).start()
        return EmptyResponse()

    def _ask_user_for_goal(self):
        try:
            x_str = input("Введите координату X цели (м) [от -5 до 5]: ")
            y_str = input("Введите координату Y цели (м) [от -5 до 5]: ")

            x = float(x_str)
            y = float(y_str)

            # Check if goal is within map bounds
            if not (self.map_origin.x <= x <= self.map_origin.x + self.map_width * self.grid_size and
                            self.map_origin.y <= y <= self.map_origin.y + self.map_height * self.grid_size):
                rospy.logerr(f"Координаты ({x:.2f}, {y:.2f}) вне допустимого диапазона карты.")
                return

            self.goal_pose = Point(x, y, 0)
            rospy.loginfo(f"Новая цель для D* Lite установлена в X={x:.2f}, Y={y:.2f} м.")
            self.initialize_dstar_lite()
            self.compute_shortest_path()
        except ValueError:
            rospy.logerr("Неверный ввод. Пожалуйста, введите числовые значения для X и Y.")
        except Exception as e:
            rospy.logerr(f"Произошла ошибка при вводе координат: {e}")

    def go_home_callback(self, req):
        rospy.loginfo(f"Вызван сервис /go_home_dstar. Устанавливаю цель в домашнюю позицию: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f})")
        self.goal_pose = Point(self.home_pose.x, self.home_pose.y, self.home_pose.z)
        self.initialize_dstar_lite()
        self.compute_shortest_path()
        return EmptyResponse()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # --- D* Lite Core Functions ---

    def initialize_dstar_lite(self):
        # Reset D* Lite state
        self.g = {}
        self.rhs = {}
        self.U = [] # Priority Queue
        self.k_m = 0
        
        start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        goal_node = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # Initialize g and rhs for all nodes to infinity
        for y in range(self.map_height):
            for x in range(self.map_width):
                node = (x, y)
                self.g[node] = float('inf')
                self.rhs[node] = float('inf')
        
        self.rhs[goal_node] = 0 # Cost from goal to itself is 0
        heappush(self.U, (self.calculate_key(goal_node), goal_node))
        
        self.last_start_node = start_node # Important for k_m updates

    def get_cost(self, u, v):
        # Cost from node u to node v (or vice versa, assuming symmetric costs)
        # 100 for occupied cells, 50 for inflated cells, 1 for free cells
        if self.map[v[1]][v[0]] == 100: # Full obstacle
            return float('inf')
        elif self.map[v[1]][v[0]] == 50: # Inflated obstacle (higher cost to discourage)
            return math.hypot(u[0] - v[0], u[1] - v[1]) * 5 # Higher cost
        else: # Free space
            return math.hypot(u[0] - v[0], u[1] - v[1]) # Euclidean distance

    def heuristic(self, s, goal):
        # Euclidean heuristic
        return math.hypot(s[0] - goal[0], s[1] - goal[1])

    def calculate_key(self, s):
        # Key = (min(g(s), rhs(s)) + h(s_start, s) + k_m, min(g(s), rhs(s)))
        # Here, h(s_start, s) is heuristic from robot's current position to s
        start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        val1 = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf'))) + self.heuristic(s, start_node) + self.k_m
        val2 = min(self.g.get(s, float('inf')), self.rhs.get(s, float('inf')))
        return (val1, val2)

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
                neighbors.append((nx, ny))
        return neighbors

    def update_vertex(self, u):
        goal_node = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)
        if u != goal_node:
            # Recompute rhs(u) by looking at its neighbors
            # rhs(u) = min_{v in succ(u)} (g(v) + cost(u, v))
            min_rhs = float('inf')
            for v_neighbor in self.get_neighbors(u):
                if self.g.get(v_neighbor, float('inf')) != float('inf'): # Only consider known paths
                    min_rhs = min(min_rhs, self.g.get(v_neighbor, float('inf')) + self.get_cost(u, v_neighbor))
            self.rhs[u] = min_rhs

        # Remove u from U if it was there
        self.U = [item for item in self.U if item[1] != u]
        heappop(self.U) # Remove the actual node if it's the top. A more efficient way is to mark as stale

        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self):
        if self.goal_pose is None:
            return

        start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        goal_node = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # Update k_m if robot has moved
        if start_node != self.last_start_node:
            self.k_m += self.heuristic(self.last_start_node, start_node)
            self.last_start_node = start_node
            # No need to update keys in U, they automatically incorporate k_m

        while self.U and (self.U[0][0] < self.calculate_key(start_node) or \
                          self.rhs.get(start_node, float('inf')) != self.g.get(start_node, float('inf'))):
            
            k_old, u = heappop(self.U)
            k_new = self.calculate_key(u)

            if k_old < k_new:
                heappush(self.U, (k_new, u))
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for s_pred in self.get_neighbors(u): # For all predecessors (neighbors of u)
                    self.update_vertex(s_pred)
            else: # g(u) <= rhs(u)
                self.g[u] = float('inf') # Mark as infinite to re-evaluate
                self.update_vertex(u) # Re-evaluate u
                for s_pred in self.get_neighbors(u): # For all predecessors (neighbors of u)
                    self.update_vertex(s_pred) # Re-evaluate predecessors

        # Reconstruct path after finding shortest path
        self.reconstruct_path()

    def update_node(self, node):
        # Call this when an edge cost changes (e.g., obstacle appears/disappears)
        if node not in self.g: # Initialize if new
            self.g[node] = float('inf')
            self.rhs[node] = float('inf')
        self.update_vertex(node) # Inform D* Lite about change at this node
        
        # Also update all neighbors of the changed node
        for neighbor in self.get_neighbors(node):
             if neighbor not in self.g: # Initialize if new
                self.g[neighbor] = float('inf')
                self.rhs[neighbor] = float('inf')
             self.update_vertex(neighbor)


    def reconstruct_path(self):
        start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        goal_node = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)
        
        current_node = start_node
        path = [current_node]

        if self.g.get(start_node, float('inf')) == float('inf'):
            rospy.logwarn("D* Lite: Путь от текущей позиции не найден.")
            self.path = []
            self.publish_current_path()
            return
            
        while current_node != goal_node:
            min_cost = float('inf')
            next_node = None
            
            # Find neighbor with min (cost_to_neighbor + g_value_of_neighbor)
            for neighbor in self.get_neighbors(current_node):
                cost = self.get_cost(current_node, neighbor)
                if self.g.get(neighbor, float('inf')) != float('inf') and \
                   cost != float('inf'): # Ensure neighbor is reachable and has a known g-value
                    
                    candidate_cost = self.g.get(neighbor, float('inf')) + cost
                    if candidate_cost < min_cost:
                        min_cost = candidate_cost
                        next_node = neighbor
            
            if next_node is None:
                rospy.logwarn("D* Lite: Не могу найти следующий узел для реконструкции пути. Возможно, заблокирован.")
                self.path = []
                break # Path blocked or corrupted
            
            path.append(next_node)
            current_node = next_node

            if len(path) > (self.map_width * self.map_height): # Safety break for infinite loops
                rospy.logerr("D* Lite: Превышен лимит размера пути при реконструкции. Возможно, ошибка в алгоритме.")
                self.path = []
                break

        self.path = path
        self.publish_current_path()
        rospy.loginfo(f"D* Lite: Путь реконструирован. Длина: {len(self.path)} точек.")


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

    # --- Path Following Logic ---

    def follow_path(self):
        cmd = Twist()

        if self.obstacle_detected_front:
            rospy.logwarn("D* Lite: Препятствие впереди! Останавливаюсь.")
            self.cmd_pub.publish(cmd) # Stop
            # D* Lite will replan automatically based on LIDAR changes
            return

        if not self.path:
            self.cmd_pub.publish(cmd)
            return

        # Check if goal is reached
        goal_dist = math.hypot(self.goal_pose.x - self.robot_pose.x, self.goal_pose.y - self.robot_pose.y)
        if goal_dist < 0.15: # Threshold for reaching goal
            self.path = []
            rospy.loginfo("D* Lite: Цель достигнута!")
            self.cmd_pub.publish(cmd)
            self.goal_pose = None
            self.path_pub.publish(Path()) # Clear path visualization
            return

        # Pop points that have been passed
        while len(self.path) > 1:
            current_gx, current_gy = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
            path_start_gx, path_start_gy = self.path[0]
            
            # Check if robot is close to or past the current path point
            dist_to_path_point = math.hypot(self.robot_pose.x - self.grid_to_world(path_start_gx, path_start_gy)[0],
                                            self.robot_pose.y - self.grid_to_world(path_start_gx, path_start_gy)[1])
            
            # Use a slightly larger threshold for popping, to allow robot to actually pass it
            if dist_to_path_point < 0.2: # If within 20cm of first path point, pop it
                self.path.pop(0)
                if self.path:
                    rospy.loginfo_throttle(1, f"D* Lite: Переход к следующей точке пути. Осталось: {len(self.path)}")
                    self.publish_current_path()
                else: # Path might become empty if goal is reached or path truncated
                    break
            else:
                break # Not close enough to pop current point

        if not self.path: # Path might become empty after popping
            return

        # Target the next point in the path (which is now self.path[0])
        target_gx, target_gy = self.path[0]
        target_wx, target_wy = self.grid_to_world(target_gx, target_gy)

        dx = target_wx - self.robot_pose.x
        dy = target_wy - self.robot_pose.y
        
        angle_to_target = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(angle_to_target - self.robot_yaw)

        # Basic P-control for angular velocity
        angular_gain = 3.0 # Aggressive gain
        cmd.angular.z = np.clip(angular_gain * yaw_error, -self.angular_speed, self.angular_speed)
        
        # Linear speed control: slow down if large angle error
        if abs(yaw_error) > math.radians(30):
            cmd.linear.x = self.linear_speed * 0.2 # Slow down significantly
        else:
            cmd.linear.x = self.linear_speed # Full speed

        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.loginfo("Ожидание odometry для инициализации позы робота для D* Lite...")
        while not rospy.is_shutdown() and \
              (self.robot_pose.x == 0 and self.robot_pose.y == 0 and self.robot_yaw == 0):
            rospy.loginfo_throttle(1, "Ожидание данных одометрии для D* Lite...")
            self.rate.sleep()
        
        # Initialize D* Lite structures after robot pose is known
        self.last_start_node = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)

        rospy.loginfo("Начальная поза D* Lite получена. Робот готов к движению.")
        rospy.loginfo("Для установки новой цели для D* Lite вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /set_goal_dstar std_srvs/Empty")
        rospy.loginfo("Затем введите координаты X и Y в терминале, где запущен этот узел.")
        rospy.loginfo("Для возвращения в домашнюю позицию (где был запущен) вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /go_home_dstar std_srvs/Empty")

        while not rospy.is_shutdown():
            if self.goal_pose and \
               (rospy.Time.now() - self.last_plan_time > self.replan_interval):
                self.compute_shortest_path()
                self.last_plan_time = rospy.Time.now()
            
            self.follow_path()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = DStarLitePlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass