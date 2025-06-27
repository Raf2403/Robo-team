#!/usr/bin/env python3
import heapq
import math

def heuristic(a, b):
    """Эвристика: евклидово расстояние"""
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def a_star(grid, start, goal, width, height):
    """A* для планирования пути на сетке
    - grid: массив оккупации (0 - свободно, >50 - препятствие)
    - start, goal: координаты в клетках
    - width, height: размер сетки в клетках
    """
    open_set = [(0, start)]  # (f_score, position)
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # 4 направления
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width:
                if grid[neighbor[0] * width + neighbor[1]] > 50:  # Препятствие
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # Путь не найден

# Пример для тестирования (настроен под ваш мир)
width = 210  # 21 м / 0.1 м = 210 клеток по X
height = 150  # 15 м / 0.1 м = 150 клеток по Y
grid = [0] * (width * height)
# Добавьте примерные препятствия (например, стены)
for i in range(50, 70):  # Простая стена в диапазоне Y
    grid[i * width + 100] = 100  # Стена на X=10 м
start = (0, 0)  # Начало в клетках (соответствует (-13, -8) в метрах)
goal = (140, 125)  # Цель в клетках (соответствует (7.4, 4.6) в метрах)
path = a_star(grid, start, goal, width, height)
print("Путь:", path)