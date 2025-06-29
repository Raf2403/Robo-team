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

class PIDController:
    """
    Простой PID контроллер.
    """
    def __init__(self, kp, ki, kd, output_min, output_max, integral_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max # Ограничение для anti-windup

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def calculate(self, error, current_time):
        """
        Вычисляет выходное значение PID.
        :param error: Текущая ошибка.
        :param current_time: Текущее время (rospy.Time.now()).
        :return: Выходное значение PID, ограниченное min/max.
        """
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            return 0.0

        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            return 0.0

        # Пропорциональный член
        p_term = self.kp * error

        # Интегральный член с anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        i_term = self.ki * self.integral

        # Дифференциальный член
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        self.last_error = error
        self.last_time = current_time

        return np.clip(output, self.output_min, self.output_max)

    def reset(self):
        """
        Сбрасывает состояние PID контроллера.
        """
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner')

        # Параметры карты
        self.grid_size = 0.1
        self.map_width = 200
        self.map_height = 200
        self.map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.map_origin = Point(-10, -10, 0)

        # Радиус "раздувания" препятствий в ячейках
        self.inflation_radius_cells = 5 # Увеличено с 3

        # Состояние робота и цели
        self.robot_pose = Point(0, 0, 0)
        self.robot_yaw = 0
        self.current_linear_vel = 0.0 # Добавлено для отслеживания текущей линейной скорости
        self.current_angular_vel = 0.0 # Добавлено для отслеживания текущей угловой скорости
        self.goal_pose = None
        self.path = []
        self.home_pose = Point(0, 0, 0)

        # ROS подписчики и публикации
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)
        
        # ROS сервисы
        rospy.Service('/set_goal', Empty, self.set_goal_callback)
        rospy.Service('/go_home', Empty, self.go_home_callback)

        # Базовые параметры движения
        self.base_linear_speed = 0.55 # Максимальная линейная скорость
        self.base_angular_speed = 1.5 # Максимальная угловая скорость
        self.rate = rospy.Rate(100) # Увеличено с 70

        self.last_plan_time = rospy.Time.now()

        # Параметры для обнаружения застревания
        self.last_robot_pose_check = Point(0,0,0)
        self.last_stuck_check_time = rospy.Time.now()
        self.stuck_threshold_distance = 0.04
        self.stuck_check_interval = rospy.Duration(1.0)

        # Параметры для активного избегания столкновений
        self.critical_obstacle_distance = 0.22
        self.safety_buffer_distance = 0.35
        self.is_avoiding_obstacle = False
        self.last_lidar_data = None

        # Индексы лучей лидара для определения передних, левых и правых препятствий
        self.obstacle_detection_angles = {
            'front': math.radians(20), # +/- 20 градусов от центра
            'side': math.radians(60)   # +/- 60 градусов от 90/-90 градусов
        }
        self.front_ranges_idx = []
        self.left_ranges_idx = []
        self.right_ranges_idx = []

        # --- PID контроллеры ---
        # PID для угловой скорости (выравнивание по углу к следующей точке пути)
        self.angular_pid = PIDController(
            kp=3.5,  # Увеличено с 2.0
            ki=0.15, # Увеличено с 0.05
            kd=0.2,  # Увеличено с 0.1
            output_min=-self.base_angular_speed,
            output_max=self.base_angular_speed,
            integral_max=1.8 # Увеличено для anti-windup
        )

        # PID для линейной скорости (замедление при приближении к цели или повороту)
        self.linear_pid = PIDController(
            kp=1.8, # Увеличено с 1.5
            ki=0.01, # Очень небольшое Ki
            kd=0.08, # Увеличено с 0.05
            output_min=0.0, # Линейная скорость всегда должна быть >= 0
            output_max=self.base_linear_speed,
            integral_max=0.0 # Интеграл не используется для линейной скорости
        )

    def world_to_grid(self, x, y):
        """Преобразует мировые координаты в координаты сетки карты."""
        gx = int((x - self.map_origin.x) / self.grid_size)
        gy = int((y - self.map_origin.y) / self.grid_size)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Преобразует координаты сетки карты в мировые координаты."""
        wx = gx * self.grid_size + self.map_origin.x + self.grid_size / 2.0
        wy = gy * self.grid_size + self.map_origin.y + self.grid_size / 2.0
        return wx, wy

    def lidar_callback(self, data):
        """Обработчик данных с LIDAR'а: обновляет карту препятствий и определяет ближайшие препятствия."""
        self.last_lidar_data = data

        # Инициализация индексов лучей LIDAR при первом получении данных
        if not self.front_ranges_idx:
            num_ranges = len(data.ranges)
            angle_per_idx = data.angle_increment

            for i in range(num_ranges):
                angle_from_front = self.normalize_angle(data.angle_min + i * angle_per_idx)
                
                # Передняя зона
                if abs(angle_from_front) < self.obstacle_detection_angles['front']:
                    self.front_ranges_idx.append(i)
                
                # Левая зона (от 90 градусов)
                # Важно: используйте нормализацию угла для сравнения
                if abs(self.normalize_angle(angle_from_front - math.radians(90))) < self.obstacle_detection_angles['side']:
                    self.left_ranges_idx.append(i)
                
                # Правая зона (от -90 градусов)
                elif abs(self.normalize_angle(angle_from_front - math.radians(-90))) < self.obstacle_detection_angles['side']:
                    self.right_ranges_idx.append(i)

        # Определение минимального расстояния до препятствия спереди
        min_front_range = self._get_avg_range(data.ranges, self.front_ranges_idx, fallback=data.range_max)

        # Логика определения препятствия впереди и запуск экстренного маневра
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

        # Обновление карты препятствий
        self.map.fill(0) # Очищаем карту
        temp_obstacles = []
        for i, r in enumerate(data.ranges):
            if data.range_min < r < data.range_max:
                angle = data.angle_min + i * data.angle_increment + self.robot_yaw
                x = self.robot_pose.x + r * math.cos(angle)
                y = self.robot_pose.y + r * math.sin(angle)
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                    temp_obstacles.append((gx, gy))

        # Наносим препятствия и "раздуваем" их
        for ox, oy in temp_obstacles:
            self.map[oy][ox] = 100 # Препятствие
            self.inflate_obstacles(ox, oy, self.inflation_radius_cells)

        self.publish_map()

    def inflate_obstacles(self, ox, oy, inflation_radius_cells):
        """Раздувает препятствия на карте."""
        for i in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for j in range(-inflation_radius_cells, inflation_radius_cells + 1):
                nx, ny = ox + i, oy + j
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if self.map[ny][nx] == 0: # Если ячейка свободна, делаем ее "почти препятствием"
                        self.map[ny][nx] = 50

    def odom_callback(self, data):
        """Обработчик данных одометрии: обновляет позу робота и скорости."""
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_linear_vel = data.twist.twist.linear.x # Обновление текущей линейной скорости
        self.current_angular_vel = data.twist.twist.angular.z # Обновление текущей угловой скорости

        # Установка домашней позиции при первом получении одометрии
        if self.home_pose.x == 0 and self.home_pose.y == 0 and self.home_pose.z == 0 and \
           (self.robot_pose.x != 0 or self.robot_pose.y != 0):
            rospy.loginfo(f"Домашняя позиция установлена в: ({self.robot_pose.x:.2f}, {self.robot_pose.y:.2f})")
            self.home_pose = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)

    def publish_map(self):
        """Публикует текущую OccupancyGrid карту."""
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
        """Коллбэк для сервиса установки новой цели."""
        rospy.loginfo("Сервис /set_goal вызван. Ожидание ввода координат цели в терминале.")
        threading.Thread(target=self._ask_user_for_goal).start()
        return EmptyResponse()

    def _ask_user_for_goal(self):
        """Запрашивает у пользователя координаты цели в терминале."""
        try:
            x_str = input("Введите координату X цели (м) [от -5 до 5]: ")
            y_str = input("Введите координату Y цели (м) [от -5 до 5]: ")

            x = float(x_str)
            y = float(y_str)

            # Проверка, что цель находится в пределах карты
            if not (self.map_origin.x <= x <= self.map_origin.x + self.map_width * self.grid_size and
                            self.map_origin.y <= y <= self.map_origin.y + self.map_height * self.grid_size):
                rospy.logerr(f"Координаты ({x:.2f}, {y:.2f}) вне допустимого диапазона карты "
                             f"[{self.map_origin.x:.2f}, {self.map_origin.x + self.map_width * self.grid_size:.2f}] "
                             f"по X и [{self.map_origin.y:.2f}, {self.map_origin.y + self.map_height * self.grid_size:.2f}] по Y.")
                return

            self.goal_pose = Point(x, y, 0)
            rospy.loginfo(f"Новая цель установлена в X={x:.2f}, Y={y:.2f} м.")
            self.plan_path()
            self.angular_pid.reset() # Сброс PID при установке новой цели
            self.linear_pid.reset()
        except ValueError:
            rospy.logerr("Неверный ввод. Пожалуйста, введите числовые значения для X и Y.")
        except Exception as e:
            rospy.logerr(f"Произошла ошибка при вводе координат: {e}")

    def go_home_callback(self, req):
        """Коллбэк для сервиса возвращения в домашнюю позицию."""
        rospy.loginfo(f"Вызван сервис /go_home. Устанавливаю цель в домашнюю позицию: ({self.home_pose.x:.2f}, {self.home_pose.y:.2f})")
        self.goal_pose = Point(self.home_pose.x, self.home_pose.y, self.home_pose.z)
        self.plan_path()
        self.angular_pid.reset() # Сброс PID при возвращении домой
        self.linear_pid.reset()
        return EmptyResponse()

    def heuristic(self, a, b):
        """Эвристическая функция (Евклидово расстояние) для A*."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors(self, node):
        """Возвращает соседние ячейки, доступные для перемещения."""
        x, y = node
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1), # По горизонтали/вертикали
            (1, 1), (-1, 1), (1, -1), (-1, -1) # По диагонали
        ]
        neighbors = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                # Разрешаем движение через слабозанятые ячейки (50), избегаем только полностью занятые (100)
                if self.map[ny, nx] < 100:
                    neighbors.append((nx, ny))
        return neighbors

    def plan_path(self):
        """Выполняет планирование глобального пути с использованием алгоритма A*."""
        if self.goal_pose is None:
            rospy.loginfo("Цель не установлена, планирование пути невозможно.")
            self.path = []
            return

        start = self.world_to_grid(self.robot_pose.x, self.robot_pose.y)
        goal = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # Проверка валидности начальной и конечной точки
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
        heappush(open_set, (0, start)) # (f_score, node)

        came_from = {} # Для восстановления пути
        g_score = {start: 0} # Стоимость от старта до текущего узла
        f_score = {start: self.heuristic(start, goal)} # g_score + heuristic

        while open_set:
            current_f, current = heappop(open_set)

            if current == goal:
                raw_path = []
                while current in came_from:
                    raw_path.append(current)
                    current = came_from[current]
                raw_path.append(start)
                raw_path.reverse()
                self.path = self.smooth_path(raw_path) # Сглаживание пути
                rospy.loginfo("Путь успешно построен.")
                self.publish_current_path()
                return

            for neighbor in self.get_neighbors(current):
                cost_to_neighbor = math.hypot(neighbor[0] - current[0], neighbor[1] - current[1])
                tentative_g = g_score[current] + cost_to_neighbor

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Путь не найден.")
        self.path = []

    def smooth_path(self, path):
        """Сглаживает путь, удаляя промежуточные точки на прямых участках."""
        if len(path) < 3:
            return path
        
        smoothed_path = [path[0]]
        i = 0
        while i < len(path) - 2:
            p1 = path[i]
            p2 = path[i+1]
            p3 = path[i+2]

            # Преобразуем в мировые координаты для расчета площади треугольника
            wx1, wy1 = self.grid_to_world(*p1)
            wx2, wy2 = self.grid_to_world(*p2)
            wx3, wy3 = self.grid_to_world(*p3)

            # Площадь треугольника, образованного тремя точками
            # Если площадь очень мала, точки почти коллинеарны
            area = abs((wx2 - wx1) * (wy3 - wy1) - (wx3 - wx1) * (wy2 - wy1))

            if area < 0.02: # Увеличено с 0.008 для более агрессивного сглаживания
                i += 1
            else:
                smoothed_path.append(path[i+1])
                i += 1
        
        smoothed_path.append(path[-1]) # Добавляем последнюю точку
        return smoothed_path

    def publish_current_path(self):
        """Публикует текущий запланированный путь для визуализации в RViz."""
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

    # --- МЕТОДЫ ДЛЯ АКТИВНОГО ИЗБЕГАНИЯ ПРЕПЯТСТВИЙ И СЛЕДОВАНИЯ ПО ПУТИ ---

    def emergency_avoidance(self):
        """
        Экстренный маневр: отъезд назад и поворот, если робот слишком близко к препятствию.
        После маневра пытается перепланировать путь.
        """
        rospy.loginfo("!!! ЭКСТРЕННЫЙ МАНЕВР ИЗБЕГАНИЯ: Опасная близость к препятствию !!!")
        cmd = Twist()

        # 1. Мгновенная остановка
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1)

        # 2. Отъезд назад
        cmd.linear.x = -0.35 # Скорость заднего хода
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(1.0) # Увеличено время движения назад

        # 3. Поворот в свободную сторону
        cmd.linear.x = 0.0
        turn_direction = 0 # 1 для влево, -1 для вправо

        if self.last_lidar_data:
            left_avg_dist = self._get_avg_range(self.last_lidar_data.ranges, self.left_ranges_idx, fallback=self.last_lidar_data.range_max)
            right_avg_dist = self._get_avg_range(self.last_lidar_data.ranges, self.right_ranges_idx, fallback=self.last_lidar_data.range_max)

            # Выбираем более свободную сторону для поворота
            if left_avg_dist > right_avg_dist and left_avg_dist > self.safety_buffer_distance:
                turn_direction = 1 # Поворот влево
                rospy.loginfo(f"ЭКСТРЕННЫЙ МАНЕВР: Поворачиваю влево. Left:{left_avg_dist:.2f}, Right:{right_avg_dist:.2f}")
            elif right_avg_dist > left_avg_dist and right_avg_dist > self.safety_buffer_distance:
                turn_direction = -1 # Поворот вправо
                rospy.loginfo(f"ЭКСТРЕННЫЙ МАНЕВР: Поворачиваю вправо. Left:{left_avg_dist:.2f}, Right:{right_avg_dist:.2f}")
            else:
                # Если обе стороны заблокированы или нет явного преимущества, выбираем случайное направление
                turn_direction = 1 if np.random.rand() > 0.5 else -1
                rospy.logwarn("ЭКСТРЕННЫЙ МАНЕВР: Оба направления заблокированы, случайный поворот.")
        else:
            turn_direction = 1 if np.random.rand() > 0.5 else -1

        cmd.angular.z = turn_direction * self.base_angular_speed * 1.2 # Полная угловая скорость для быстрого поворота
        self.cmd_pub.publish(cmd)
        rospy.sleep(1.2) # Увеличено время поворота

        # 4. Финальная остановка
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.2)

        rospy.loginfo("Экстренный маневр завершен. Пытаюсь перепланировать.")
        self.is_avoiding_obstacle = False
        self.plan_path()
        self.angular_pid.reset() # Сброс PID после экстренного маневра
        self.linear_pid.reset()

    def follow_path_and_avoid(self):
        """
        Основной контроллер, который следует по глобальному пути,
        но реактивно объезжает препятствия, если они близко.
        Использует PID для более точного управления.
        """
        cmd = Twist()

        if self.is_avoiding_obstacle:
            # Если робот в режиме экстренного маневра, не вмешиваемся в управление
            return

        if not self.path:
            # Если пути нет, останавливаемся
            self.cmd_pub.publish(cmd)
            # Добавим логику для остановки PID, если нет пути
            self.angular_pid.reset()
            self.linear_pid.reset()
            return

        # Проверка достижения конечной цели
        if self.goal_pose:
            goal_dist = math.hypot(self.goal_pose.x - self.robot_pose.x, self.goal_pose.y - self.robot_pose.y)
            # *** ИЗМЕНЕНИЕ 1: Увеличение порога достижения цели ***
            if goal_dist < 0.15: # Увеличиваем порог достижения цели с 0.08 до 0.15 для меньшего "мешкания"
                self.path = []
                rospy.loginfo("Цель достигнута!")
                self.cmd_pub.publish(cmd)
                self.goal_pose = None
                self.angular_pid.reset() # Сброс PID при достижении цели
                self.linear_pid.reset()
                return

        # --- Локальная логика избегания/Wall Following ---
        if self.obstacle_in_front:
            rospy.loginfo_throttle(0.1, "Препятствие впереди. Включаю режим объезда.")
            if self.last_lidar_data:
                ranges = self.last_lidar_data.ranges

                front_avg_dist = self._get_avg_range(ranges, self.front_ranges_idx, self.last_lidar_data.range_max)
                left_avg_dist = self._get_avg_range(ranges, self.left_ranges_idx, self.last_lidar_data.range_max)
                right_avg_dist = self._get_avg_range(ranges, self.right_ranges_idx, self.last_lidar_data.range_max)

                turn_angular_z = 0.0
                
                if front_avg_dist > self.safety_buffer_distance:
                    cmd.linear.x = self.base_linear_speed * 0.8
                    if left_avg_dist > right_avg_dist:
                        turn_angular_z = self.base_angular_speed * 0.1
                    else:
                        turn_angular_z = -self.base_angular_speed * 0.1
                elif front_avg_dist > self.critical_obstacle_distance + 0.05:
                    cmd.linear.x = self.base_linear_speed * 0.4
                    if left_avg_dist > right_avg_dist:
                        turn_angular_z = self.base_angular_speed * 0.7
                    else:
                        turn_angular_z = -self.base_angular_speed * 0.7
                else:
                    cmd.linear.x = 0.0
                    if left_avg_dist > right_avg_dist:
                        turn_angular_z = self.base_angular_speed * 1.0
                    elif right_avg_dist > left_avg_dist:
                        turn_angular_z = -self.base_angular_speed * 1.0
                    else:
                        turn_angular_z = self.base_angular_speed * 1.0 # Вращение на месте

                cmd.angular.z = np.clip(turn_angular_z, -self.base_angular_speed, self.base_angular_speed)
                
                self.cmd_pub.publish(cmd)
                # Сброс PID при переключении на локальное объезжание
                self.angular_pid.reset()
                self.linear_pid.reset()
                return

        # --- Следование по глобальному пути с PID ---
        target_gx, target_gy = self.path[0]
        current_waypoint_world_x, current_waypoint_world_y = self.grid_to_world(target_gx, target_gy)
        distance_to_current_waypoint = math.hypot(current_waypoint_world_x - self.robot_pose.x,
                                                 current_waypoint_world_y - self.robot_pose.y)
        
        # *** ИЗМЕНЕНИЕ 2: Увеличение порога перехода к следующей точке пути ***
        # Если текущая точка пути достигнута (или почти достигнута), переходим к следующей
        if distance_to_current_waypoint < 0.2: # Увеличено с 0.1 для более плавного перехода
            if len(self.path) > 1:
                self.path.pop(0)
                rospy.loginfo(f"Переход к следующей точке пути. Осталось: {len(self.path)}")
                self.publish_current_path()
                self.angular_pid.reset() # Сброс PID при переходе к новой точке для более чистого старта
                self.linear_pid.reset()
                # Обновим целевую точку, если перешли к следующей
                if self.path:
                    target_gx, target_gy = self.path[0]
                    current_waypoint_world_x, current_waypoint_world_y = self.grid_to_world(target_gx, target_gy)
            else:
                # Если это последняя точка пути, но конечная цель еще не достигнута по глобальной проверке, ждем.
                pass 

        # Если после pop путь пуст, но цель еще не достигнута, перепланируем
        if not self.path and self.goal_pose:
            rospy.logwarn_throttle(5, "Путь закончился, но цель не достигнута. Перепланирование...")
            self.plan_path()
            return

        # Если путь есть, вычисляем скорости
        if self.path:
            target_gx, target_gy = self.path[0]
            target_wx, target_wy = self.grid_to_world(target_gx, target_gy)
            dx = target_wx - self.robot_pose.x
            dy = target_wy - self.robot_pose.y
            
            angle_to_waypoint = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(angle_to_waypoint - self.robot_yaw)

            # Расчет угловой скорости с PID
            cmd.angular.z = self.angular_pid.calculate(yaw_error, rospy.Time.now())
            
            # Расчет желаемой линейной скорости
            desired_linear_speed = self.base_linear_speed

            # Уменьшаем скорость при большом угле поворота (угловая ошибка)
            angular_speed_factor = max(0.0, 1.0 - abs(yaw_error) / math.radians(60)) # Снижаем до 0 при 60 градусах
            desired_linear_speed *= angular_speed_factor
            
            # Замедляемся, если расстояние до конечной цели меньше 1.5м (для плавного прибытия)
            if self.goal_pose:
                distance_to_final_goal = math.hypot(self.goal_pose.x - self.robot_pose.x, self.goal_pose.y - self.robot_pose.y)
                # *** ИЗМЕНЕНИЕ 3: Более агрессивное замедление к конечной цели ***
                if distance_to_final_goal < 1.0: # Изменено с 1.5 на 1.0 для начала замедления поближе
                    # Линейная функция замедления: чем ближе, тем меньше скорость
                    distance_factor = distance_to_final_goal / 1.0 # Нормализуем расстояние от 0 до 1
                    # Минимум 10% от желаемой скорости, чтобы не останавливаться слишком рано
                    desired_linear_speed *= np.clip(distance_factor, 0.1, 1.0) 
                
                # *** ИЗМЕНЕНИЕ 4: Специальная фаза прибытия для точного выравнивания и остановки ***
                if distance_to_final_goal < 0.3: # Если очень близко к цели (например, 30 см)
                    rospy.loginfo_throttle(0.5, "Вход в фазу прибытия: точное выравнивание.")
                    # Задаем очень маленькую линейную скорость, чтобы дать время на поворот
                    desired_linear_speed = 0.05 # Очень низкая скорость
                    # Если угловая ошибка мала, можно совсем остановить линейное движение
                    if abs(yaw_error) < math.radians(5): # В пределах 5 градусов
                        desired_linear_speed = 0.01 # Почти остановка

            # Используем linear_pid для регулировки скорости на основе ошибки: (желаемая скорость - текущая скорость)
            linear_error = desired_linear_speed - self.current_linear_vel
            cmd.linear.x = self.linear_pid.calculate(linear_error, rospy.Time.now())
            cmd.linear.x = np.clip(cmd.linear.x, 0.0, self.base_linear_speed) # Ограничиваем скорость

            self.cmd_pub.publish(cmd)
        else:
            # Если пути нет, останавливаемся
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

    def _get_avg_range(self, ranges, indices, fallback=float('inf')):
        """
        Вспомогательная функция для получения среднего значения дальности по выбранным индексам LIDAR.
        Возвращает fallback значение, если нет действительных измерений.
        """
        valid_ranges = []
        for idx in indices:
            if 0 <= idx < len(ranges):
                r = ranges[idx]
                # Проверяем, что значение находится в допустимом диапазоне LIDAR
                if self.last_lidar_data.range_min < r < self.last_lidar_data.range_max:
                    valid_ranges.append(r)
        if valid_ranges:
            return np.mean(valid_ranges)
        return fallback # Возвращаем значение по умолчанию, если нет валидных данных
        
    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def run(self):
        """Основной цикл работы планировщика."""
        rospy.loginfo("Ожидание odometry для получения начальной позы робота...")
        # Ждем, пока робот не получит свои начальные координаты
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
                # Если активен экстренный маневр, просто ждем его завершения
                self.rate.sleep()
                continue 

            # Обнаружение застревания
            if self.goal_pose and (rospy.Time.now() - self.last_stuck_check_time > self.stuck_check_interval):
                dist_moved = math.hypot(self.robot_pose.x - self.last_robot_pose_check.x,
                                             self.robot_pose.y - self.last_robot_pose_check.y)
                if dist_moved < self.stuck_threshold_distance:
                    rospy.logwarn("Робот, возможно, застрял! Запускаю экстренный маневр.")
                    self.is_avoiding_obstacle = True
                    threading.Thread(target=self.emergency_avoidance).start()
                self.last_robot_pose_check = Point(self.robot_pose.x, self.robot_pose.y, self.robot_pose.z)
                self.last_stuck_check_time = rospy.Time.now()

            # Глобальное перепланирование пути
            # Перепланируем, если: нет пути, прошло достаточно времени, или есть препятствие впереди
            if self.goal_pose and (not self.path or \
               (rospy.Time.now() - self.last_plan_time > rospy.Duration(1.0)) or \
               (not self.is_avoiding_obstacle and self.obstacle_in_front)):
                rospy.loginfo_throttle(0.2, "Инициирую перепланирование глобального маршрута...")
                self.plan_path()
                self.last_plan_time = rospy.Time.now()
            
            # Основная логика следования по пути и объезда препятствий
            self.follow_path_and_avoid()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        planner = AStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
