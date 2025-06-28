#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import math
import random
import threading
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

class RRTStarPlanner:
    def __init__(self):
        rospy.init_node('rrt_star_planner')

        # Map Parameters
        self.grid_size = 0.1 # meters per cell
        self.map_width_meters = 20.0 # meters
        self.map_height_meters = 20.0 # meters
        self.map_origin = Point(-10.0, -10.0, 0) # Bottom-left corner of the map in world coords
        self.map_width_cells = int(self.map_width_meters / self.grid_size)
        self.map_height_cells = int(self.map_height_meters / self.grid_size)
        
        self.map = np.zeros((self.map_height_cells, self.map_width_cells), dtype=np.int8) 
        self.inflation_radius_cells = 4 # for collision checking in RRT*

        # RRT* Parameters
        self.max_iter = 5000 # Maximum number of iterations for tree growth (tune based on map size/complexity)
        self.goal_sample_rate = 0.1 # Probability of sampling the goal directly (0.05-0.15 is common)
        self.connect_dist = 2.0 # Max distance to extend a branch and connect to nearest neighbor (in meters)
        self.search_radius = 3.0 # Radius for finding neighbors for rewiring (in meters)
        self.min_dist_to_goal = 0.5 # Distance to consider goal reached by RRT*

        # Robot and Goal State
        self.robot_pose = Point(0, 0, 0)
        self.robot_yaw = 0
        self.goal_pose = None
        self.path = []
        self.home_pose = Point(0, 0, 0)

        # RRT* Tree Structure: {node_idx: (x, y, parent_idx, cost)}
        self.tree = {}
        self.node_count = 0

        # ROS Subscribers and Publishers
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/rrt_star_path", Path, queue_size=1)
        self.tree_pub = rospy.Publisher("/rrt_star_tree", Path, queue_size=1) # To visualize the tree
        
        # ROS Services
        rospy.Service('/set_goal_rrt', Empty, self.set_goal_callback)
        rospy.Service('/go_home_rrt', Empty, self.go_home_callback)

        # Control Parameters
        self.linear_speed = 0.4 # m/s (RRT* paths can be longer, so speed might need adjustment)
        self.angular_speed = 1.0 # rad/s
        self.rate = rospy.Rate(50) # Hz

        self.last_plan_time = rospy.Time.now()
        self.replan_interval = rospy.Duration(5.0) # RRT* is slower for replanning all, so less frequent

        # Obstacle avoidance for path following
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
        # Clear map for new obstacles
        self.map.fill(0) 
        
        min_front_range = data.range_max
        front_angle_threshold = math.radians(20)

        for i, r in enumerate(data.ranges):
            if data.range_min < r < data.range_max:
                angle_global = data.angle_min + i * data.angle_increment + self.robot_yaw
                x = self.robot_pose.x + r * math.cos(angle_global)
                y = self.robot_pose.y + r * math.sin(angle_global)
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.map_width_cells and 0 <= gy < self.map_height_cells:
                    self.map[gy][gx] = 100 # Mark as occupied
                    self.inflate_obstacles(gx, gy, self.inflation_radius_cells)

                if abs(self.normalize_angle(data.angle_min + i * data.angle_increment)) < front_angle_threshold:
                    min_front_range = min(min_front_range, r)

        self.obstacle_detected_front = min_front_range < self.critical_obstacle_distance
        self.publish_map()


    def inflate_obstacles(self, ox, oy, inflation_radius_cells):
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = ox + i, oy + j
                if 0 <= nx < self.map_width_cells and 0 <= ny < self.map_height_cells:
                    if self.map[ny][nx] == 0:
                        self.map[ny][nx] = 50 # Partially occupied for cost/collision checking

    def odom_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        if self.home_pose.x == 0 and self.home_pose.y == 0 and self.home_pose.z == 0 and \
           (self.robot_pose.x != 0 or self.robot_pose.y != 0):
            rospy.loginfo(f"Домашняя позиция для RRT* установлена в: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
            self.home_pose = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid_size
        msg.info.width = self.map_width_cells
        msg.info.height = self.map_height_cells
        msg.info.origin.position = self.map_origin
        msg.data = self.map.flatten().tolist()
        self.map_pub.publish(msg)

    def set_goal_callback(self, req):
        rospy.loginfo("Сервис /set_goal_rrt вызван. Ожидание ввода координат цели в терминале.")
        threading.Thread(target=self._ask_user_for_goal).start()
        return EmptyResponse()

    def _ask_user_for_goal(self):
        try:
            x_str = input("Введите координату X цели (м) [от -5 до 5]: ")
            y_str = input("Введите координату Y цели (м) [от -5 до 5]: ")

            x = float(x_str)
            y = float(y_str)

            if not (self.map_origin.x <= x <= self.map_origin.x + self.map_width_meters and
                            self.map_origin.y <= y <= self.map_origin.y + self.map_height_meters):
                rospy.logerr(f"Координаты ({x:.2f}, {y:.2f}) вне допустимого диапазона карты.")
                return

            self.goal_pose = Point(x, y, 0)
            rospy.loginfo(f"Новая цель для RRT* установлена в X={x:.2f}, Y={y:.2f} м.")
            self.plan_path_rrt_star()
        except ValueError:
            rospy.logerr("Неверный ввод. Пожалуйста, введите числовые значения для X и Y.")
        except Exception as e:
            rospy.logerr(f"Произошла ошибка при вводе координат: {e}")

    def go_home_callback(self, req):
        rospy.loginfo(f"Вызван сервис /go_home_rrt. Устанавливаю цель в домашнюю позицию: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f})")
        self.goal_pose = Point(self.home_pose.x, self.home_pose.y, self.home_pose.z)
        self.plan_path_rrt_star()
        return EmptyResponse()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # --- RRT* Core Functions ---

    def plan_path_rrt_star(self):
        if self.goal_pose is None:
            rospy.loginfo("RRT*: Цель не установлена.")
            self.path = []
            return

        start_x, start_y = self.robot_pose.x, self.robot_pose.y
        goal_x, goal_y = self.goal_pose.x, self.goal_pose.y

        self.tree = {0: (start_x, start_y, -1, 0.0)} # {node_idx: (x, y, parent_idx, cost)}
        self.node_count = 1

        goal_node_idx = -1 # Index of a tree node within reach of the goal

        rospy.loginfo("RRT*: Начинаю планирование...")
        for i in range(self.max_iter):
            if rospy.is_shutdown(): return

            # 1. Sample a random point (or goal with some probability)
            rand_x, rand_y = self.generate_random_point(goal_x, goal_y)

            # 2. Find the nearest node in the tree to the random point
            nearest_idx, nearest_node = self.find_nearest_node(rand_x, rand_y)

            # 3. Steer from nearest node towards random point (extend the branch)
            new_node_x, new_node_y = self.steer(nearest_node[0], nearest_node[1], rand_x, rand_y)
            
            # Check if the steered point is in collision
            if self.is_collision_free(nearest_node[0], nearest_node[1], new_node_x, new_node_y):
                # 4. Find nearby nodes for connection (within search_radius)
                nearby_nodes_indices = self.find_nearby_nodes(new_node_x, new_node_y)

                # 5. Choose parent: find the best parent from nearby nodes
                # Cost to new_node = cost to potential_parent + cost_from_parent_to_new_node
                best_parent_idx = nearest_idx
                min_cost_to_new_node = self.tree[nearest_idx][3] + math.hypot(new_node_x - nearest_node[0], new_node_y - nearest_node[1])

                for idx in nearby_nodes_indices:
                    parent_candidate = self.tree[idx]
                    cost_from_candidate = math.hypot(new_node_x - parent_candidate[0], new_node_y - parent_candidate[1])
                    if self.is_collision_free(parent_candidate[0], parent_candidate[1], new_node_x, new_node_y):
                        if parent_candidate[3] + cost_from_candidate < min_cost_to_new_node:
                            min_cost_to_new_node = parent_candidate[3] + cost_from_candidate
                            best_parent_idx = idx
                
                # Add the new node to the tree
                new_node_idx = self.node_count
                self.tree[new_node_idx] = (new_node_x, new_node_y, best_parent_idx, min_cost_to_new_node)
                self.node_count += 1

                # 6. Rewire: Try to improve paths for nearby nodes through the new node
                for idx in nearby_nodes_indices:
                    if idx == new_node_idx: continue # Don't rewire itself
                    
                    neighbor_node = self.tree[idx]
                    cost_from_new_node = math.hypot(neighbor_node[0] - new_node_x, neighbor_node[1] - new_node_y)
                    
                    if self.is_collision_free(new_node_x, new_node_y, neighbor_node[0], neighbor_node[1]):
                        if self.tree[new_node_idx][3] + cost_from_new_node < neighbor_node[3]:
                            # New path through new_node is better, rewire
                            self.tree[idx] = (neighbor_node[0], neighbor_node[1], new_node_idx, self.tree[new_node_idx][3] + cost_from_new_node)
                            # Propagate cost updates down the tree if necessary (more advanced RRT* implementations)

                # Check if goal is reached
                dist_to_goal = math.hypot(new_node_x - goal_x, new_node_y - goal_y)
                if dist_to_goal < self.min_dist_to_goal:
                    # Found a node close enough to goal, try to connect if collision-free
                    if self.is_collision_free(new_node_x, new_node_y, goal_x, goal_y):
                        # Add a temporary goal node to the tree for path reconstruction
                        goal_node_idx = new_node_idx
                        rospy.loginfo(f"RRT*: Цель достигнута после {i+1} итераций. Reconstructing path.")
                        break
            
            if (i % 500) == 0:
                rospy.loginfo(f"RRT* iteration {i}/{self.max_iter}")
        
        if goal_node_idx == -1:
            rospy.logwarn("RRT*: Путь до цели не найден после всех итераций.")
            self.path = []
        else:
            self.reconstruct_rrt_path(goal_node_idx)
        
        self.publish_rrt_tree() # Visualize the tree
        self.publish_current_path()


    def generate_random_point(self, goal_x, goal_y):
        if random.random() < self.goal_sample_rate:
            return goal_x, goal_y
        
        # Sample within map bounds
        x = random.uniform(self.map_origin.x, self.map_origin.x + self.map_width_meters)
        y = random.uniform(self.map_origin.y, self.map_origin.y + self.map_height_meters)
        return x, y

    def find_nearest_node(self, rx, ry):
        min_dist = float('inf')
        nearest_idx = -1
        nearest_node_data = None
        for idx, node_data in self.tree.items():
            nx, ny, _, _ = node_data
            dist = math.hypot(rx - nx, ry - ny)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx
                nearest_node_data = node_data
        return nearest_idx, nearest_node_data

    def steer(self, from_x, from_y, to_x, to_y):
        dist = math.hypot(to_x - from_x, to_y - from_y)
        if dist < self.connect_dist:
            return to_x, to_y # If target is close, just go to it
        
        # Extend only connect_dist towards the target
        angle = math.atan2(to_y - from_y, to_x - from_x)
        new_x = from_x + self.connect_dist * math.cos(angle)
        new_y = from_y + self.connect_dist * math.sin(angle)
        return new_x, new_y

    def is_collision_free(self, x1, y1, x2, y2):
        # Check collision along the line segment (x1,y1) to (x2,y2)
        # Iterate over points on the line and check map occupancy
        num_steps = int(math.hypot(x2 - x1, y2 - y1) / self.grid_size) # Check every grid cell
        if num_steps == 0: num_steps = 1 # Ensure at least one check for very short segments
        
        for i in range(num_steps + 1):
            ratio = i / num_steps
            check_x = x1 + ratio * (x2 - x1)
            check_y = y1 + ratio * (y2 - y1)
            
            gx, gy = self.world_to_grid(check_x, check_y)
            if not (0 <= gx < self.map_width_cells and 0 <= gy < self.map_height_cells):
                return False # Out of map bounds

            # Check for obstacles (value 100 for full obstacle, 50 for inflated area)
            if self.map[gy][gx] >= 50: 
                return False # Collision
        return True

    def find_nearby_nodes(self, x, y):
        nearby_indices = []
        for idx, node_data in self.tree.items():
            nx, ny, _, _ = node_data
            dist = math.hypot(x - nx, y - ny)
            if dist < self.search_radius:
                nearby_indices.append(idx)
        return nearby_indices

    def reconstruct_rrt_path(self, goal_node_idx):
        current_idx = goal_node_idx
        path = []
        while current_idx != -1:
            x, y, parent_idx, _ = self.tree[current_idx]
            path.append((x, y))
            current_idx = parent_idx
        path.reverse()
        self.path = path
        rospy.loginfo(f"RRT*: Путь реконструирован. Длина: {len(self.path)} точек.")

    def publish_rrt_tree(self):
        tree_msg = Path()
        tree_msg.header.stamp = rospy.Time.now()
        tree_msg.header.frame_id = "map"
        
        for idx, node_data in self.tree.items():
            x, y, parent_idx, _ = node_data
            if parent_idx != -1: # Draw a line segment from child to parent
                parent_x, parent_y, _, _ = self.tree[parent_idx]
                
                # To visualize segments, you might need to publish individual markers or use a specialized tool
                # For simplicity, we'll just publish all nodes as a 'path' and it will look like a blob.
                # A better way would be to use MarkerArray in RViz.
                
                # For now, just add nodes to visualize them.
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                tree_msg.poses.append(pose)
        self.tree_pub.publish(tree_msg)


    def publish_current_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for x, y in self.path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    # --- Path Following Logic ---

    def follow_path(self):
        cmd = Twist()

        if self.obstacle_detected_front:
            rospy.logwarn("RRT*: Препятствие впереди! Останавливаюсь.")
            self.cmd_pub.publish(cmd) # Stop
            # RRT* will replan if obstacle detected AND it's time for replan
            return

        if not self.path:
            self.cmd_pub.publish(cmd)
            return

        # Check if goal is reached
        goal_dist = math.hypot(self.goal_pose.x - self.robot_pose.x, self.goal_pose.y - self.robot_pose.y)
        if goal_dist < 0.15: # Threshold for reaching goal
            self.path = []
            rospy.loginfo("RRT*: Цель достигнута!")
            self.cmd_pub.publish(cmd)
            self.goal_pose = None
            self.path_pub.publish(Path()) # Clear path visualization
            return

        # Find the closest point on the path to the robot
        closest_point_idx = 0
        min_dist_to_path = float('inf')
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(self.robot_pose.x - px, self.robot_pose.y - py)
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_point_idx = i

        # Target the next point in the path from the closest point
        target_idx = min(closest_point_idx + 5, len(self.path) - 1) # Look a few points ahead for smoothness
        if len(self.path) == 1:
            target_idx = 0 # If only one point, target that one
            
        target_wx, target_wy = self.path[target_idx]

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
        rospy.loginfo("Ожидание odometry для инициализации позы робота для RRT*...")
        while not rospy.is_shutdown() and \
              (self.robot_pose.x == 0 and self.robot_pose.y == 0 and self.robot_yaw == 0):
            rospy.loginfo_throttle(1, "Ожидание данных одометрии для RRT*...")
            self.rate.sleep()
        
        rospy.loginfo("Начальная поза RRT* получена. Робот готов к движению.")
        rospy.loginfo("Для установки новой цели для RRT* вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /set_goal_rrt std_srvs/Empty")
        rospy.loginfo("Затем введите координаты X и Y в терминале, где запущен этот узел.")
        rospy.loginfo("Для возвращения в домашнюю позицию (где был запущен) вызовите ROS сервис:")
        rospy.loginfo("  rosservice call /go_home_rrt std_srvs/Empty")

        while not rospy.is_shutdown():
            if self.goal_pose and \
               (rospy.Time.now() - self.last_plan_time > self.replan_interval):
                rospy.loginfo_throttle(1, "RRT*: Инициирую перепланирование...")
                self.plan_path_rrt_star()
                self.last_plan_time = rospy.Time.now()
            
            self.follow_path()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = RRTStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass