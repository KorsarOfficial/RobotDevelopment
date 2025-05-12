#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Утилиты для работы с роботом.
Содержит функции преобразования координат, расчеты кинематики,
и вспомогательные инструменты для визуализации.
"""

import numpy as np
import math
import logging

logger = logging.getLogger(__name__)

def setup_logger(level=logging.INFO, log_file=None):
    """
    Настройка логгера для модуля.
    
    Args:
        level (int): Уровень логирования
        log_file (str, optional): Путь к файлу для записи логов
    """
    logger.setLevel(level)
    
    # Создаем обработчик для вывода в консоль
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # Если указан файл, добавляем обработчик для записи в файл
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    logger.info("Логгер настроен")
    return logger

def cartesian_to_joint(x, y, z, orientation):
    """
    Преобразование декартовых координат в угловые (обратная кинематика).
    Это упрощенная реализация для эмуляции.
    
    Args:
        x, y, z (float): Координаты в декартовой системе
        orientation (list): Ориентация инструмента [roll, pitch, yaw]
        
    Returns:
        list: Углы сервоприводов робота [angle1, angle2, ...]
    """
    logger.debug(f"Преобразование координат {x}, {y}, {z} с ориентацией {orientation}")
    
    # Упрощенный расчет для эмуляции (не для реального робота)
    # Для реального робота здесь должны быть формулы обратной кинематики
    
    # Расчет угла основания (в плоскости XY)
    if x == 0 and y == 0:
        base_angle = 0
    else:
        base_angle = math.degrees(math.atan2(y, x))
    
    # Радиус в плоскости XY
    r = math.sqrt(x*x + y*y)
    
    # Упрощенный расчет для угла плеча и локтя
    # В реальной реализации здесь должен быть сложный расчет кинематики
    arm_length = 100  # Условная длина плеча
    forearm_length = 100  # Условная длина предплечья
    
    # Расчет углов по закону косинусов (упрощенно)
    try:
        elevation = math.atan2(z, r)
        d = math.sqrt(r*r + z*z)
        
        if d > (arm_length + forearm_length):
            logger.warning("Точка за пределами досягаемости робота")
            return None
        
        # Угол локтя по закону косинусов
        cos_elbow = (arm_length*arm_length + forearm_length*forearm_length - d*d) / (2 * arm_length * forearm_length)
        cos_elbow = max(-1, min(1, cos_elbow))  # Ограничиваем значение в пределах [-1, 1]
        elbow_angle = math.pi - math.acos(cos_elbow)
        
        # Угол плеча
        cos_shoulder = (arm_length*arm_length + d*d - forearm_length*forearm_length) / (2 * arm_length * d)
        cos_shoulder = max(-1, min(1, cos_shoulder))  # Ограничиваем значение в пределах [-1, 1]
        shoulder_angle = elevation + math.acos(cos_shoulder)
        
        # Преобразуем углы в градусы
        shoulder_angle_deg = math.degrees(shoulder_angle)
        elbow_angle_deg = math.degrees(elbow_angle)
        
        # Упрощенная ориентация инструмента (запястья)
        roll, pitch, yaw = orientation
        
        # Возвращаем результат в виде списка углов
        joint_angles = [
            base_angle,  # Основание
            shoulder_angle_deg,  # Плечо
            elbow_angle_deg,  # Локоть
            roll,  # Запястье (поворот)
            pitch,  # Запястье (наклон)
            yaw  # Запястье (вращение)
        ]
        
        logger.debug(f"Рассчитанные углы: {joint_angles}")
        return joint_angles
        
    except Exception as e:
        logger.error(f"Ошибка расчета обратной кинематики: {e}")
        return None

def joint_to_cartesian(joint_angles):
    """
    Преобразование угловых координат в декартовы (прямая кинематика).
    Это упрощенная реализация для эмуляции.
    
    Args:
        joint_angles (list): Углы сервоприводов [angle1, angle2, ...]
        
    Returns:
        tuple: (x, y, z, [roll, pitch, yaw]) координаты и ориентация
    """
    logger.debug(f"Расчет прямой кинематики для углов: {joint_angles}")
    
    try:
        # Извлекаем углы из списка
        base_angle, shoulder_angle, elbow_angle, roll, pitch, yaw = joint_angles
        
        # Преобразуем углы в радианы
        base_rad = math.radians(base_angle)
        shoulder_rad = math.radians(shoulder_angle)
        elbow_rad = math.radians(elbow_angle)
        
        # Условные длины частей манипулятора
        arm_length = 100  # Длина плеча
        forearm_length = 100  # Длина предплечья
        
        # Расчет позиции локтя
        elbow_x = arm_length * math.cos(shoulder_rad) * math.cos(base_rad)
        elbow_y = arm_length * math.cos(shoulder_rad) * math.sin(base_rad)
        elbow_z = arm_length * math.sin(shoulder_rad)
        
        # Расчет позиции запястья
        hand_direction = shoulder_rad + elbow_rad  # Общее направление руки
        wrist_x = elbow_x + forearm_length * math.cos(hand_direction) * math.cos(base_rad)
        wrist_y = elbow_y + forearm_length * math.cos(hand_direction) * math.sin(base_rad)
        wrist_z = elbow_z + forearm_length * math.sin(hand_direction)
        
        # Ориентация инструмента
        orientation = [roll, pitch, yaw]
        
        result = (wrist_x, wrist_y, wrist_z, orientation)
        logger.debug(f"Результат прямой кинематики: {result}")
        return result
        
    except Exception as e:
        logger.error(f"Ошибка расчета прямой кинематики: {e}")
        return None

def validate_position(x, y, z, limits):
    """
    Проверка, что координаты находятся внутри рабочей зоны.
    
    Args:
        x, y, z (float): Координаты для проверки
        limits (dict): Словарь с границами рабочей зоны
        
    Returns:
        bool: True если координаты допустимы, иначе False
    """
    if (limits["x_min"] <= x <= limits["x_max"] and
        limits["y_min"] <= y <= limits["y_max"] and
        limits["z_min"] <= z <= limits["z_max"]):
        return True
    else:
        logger.warning(f"Координаты ({x}, {y}, {z}) вне рабочей зоны")
        return False

def validate_joint_angles(angles, limits):
    """
    Проверка, что углы сервоприводов находятся в допустимых пределах.
    
    Args:
        angles (list): Список углов для проверки
        limits (list): Список словарей с минимальными и максимальными значениями углов
        
    Returns:
        bool: True если углы допустимы, иначе False
    """
    if len(angles) != len(limits):
        logger.error(f"Неверное количество углов: {len(angles)}, ожидалось: {len(limits)}")
        return False
    
    for i, angle in enumerate(angles):
        if not (limits[i]["min"] <= angle <= limits[i]["max"]):
            logger.warning(f"Угол {i} ({angle}) вне допустимого диапазона [{limits[i]['min']}, {limits[i]['max']}]")
            return False
    
    return True

def interpolate_positions(start_pos, end_pos, steps):
    """
    Линейная интерполяция между двумя позициями.
    
    Args:
        start_pos (list): Начальная позиция [x, y, z]
        end_pos (list): Конечная позиция [x, y, z]
        steps (int): Количество шагов интерполяции
        
    Returns:
        list: Список точек интерполяции
    """
    path = []
    
    for i in range(steps + 1):
        t = i / steps  # Параметр интерполяции (от 0 до 1)
        
        # Линейная интерполяция координат
        x = start_pos[0] + t * (end_pos[0] - start_pos[0])
        y = start_pos[1] + t * (end_pos[1] - start_pos[1])
        z = start_pos[2] + t * (end_pos[2] - start_pos[2])
        
        path.append([x, y, z])
    
    return path

def calculate_trajectory(points, speed=50.0):
    """
    Расчет траектории и времени перемещения по точкам.
    
    Args:
        points (list): Список точек траектории [[x1, y1, z1], [x2, y2, z2], ...]
        speed (float): Скорость перемещения (единиц в секунду)
        
    Returns:
        tuple: (trajectory, time), где trajectory - список точек с временем,
                                 time - общее время перемещения
    """
    trajectory = []
    total_time = 0.0
    
    for i in range(len(points) - 1):
        # Расчет расстояния между текущей и следующей точкой
        dx = points[i+1][0] - points[i][0]
        dy = points[i+1][1] - points[i][1]
        dz = points[i+1][2] - points[i][2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Расчет времени перемещения
        segment_time = distance / speed
        total_time += segment_time
        
        # Добавляем точку с временем
        trajectory.append({
            "point": points[i],
            "time": segment_time
        })
    
    # Добавляем последнюю точку
    trajectory.append({
        "point": points[-1],
        "time": 0.0  # Время остановки в конечной точке
    })
    
    return trajectory, total_time

def rotation_matrix_to_euler(R):
    """
    Преобразование матрицы поворота в углы Эйлера.
    
    Args:
        R (numpy.ndarray): Матрица поворота 3x3
        
    Returns:
        list: Углы Эйлера [roll, pitch, yaw] в градусах
    """
    try:
        # Проверяем, что матрица правильной формы
        if R.shape != (3, 3):
            raise ValueError(f"Ожидалась матрица 3x3, получена: {R.shape}")
        
        # Извлекаем углы Эйлера из матрицы поворота
        # Используем порядок ZYX (yaw, pitch, roll)
        
        # Синус pitch
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        
        # Проверка на "gimbal lock" (блокировка карданова подвеса)
        singular = sy < 1e-6
        
        if not singular:
            roll = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], sy)
            yaw = math.atan2(R[1,0], R[0,0])
        else:
            roll = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw = 0
        
        # Преобразуем из радиан в градусы
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return [roll_deg, pitch_deg, yaw_deg]
        
    except Exception as e:
        logger.error(f"Ошибка преобразования матрицы поворота в углы Эйлера: {e}")
        return [0, 0, 0]  # Возвращаем нулевые углы в случае ошибки

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Преобразование углов Эйлера в матрицу поворота.
    
    Args:
        roll, pitch, yaw (float): Углы Эйлера в градусах
        
    Returns:
        numpy.ndarray: Матрица поворота 3x3
    """
    try:
        # Преобразуем из градусов в радианы
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Создаем матрицы поворота вокруг каждой оси
        # Поворот вокруг оси X (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll_rad), -math.sin(roll_rad)],
            [0, math.sin(roll_rad), math.cos(roll_rad)]
        ])
        
        # Поворот вокруг оси Y (pitch)
        Ry = np.array([
            [math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [0, 1, 0],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]
        ])
        
        # Поворот вокруг оси Z (yaw)
        Rz = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Составляем общую матрицу поворота
        # Порядок: сначала yaw, затем pitch, затем roll (Z-Y-X)
        R = Rz.dot(Ry).dot(Rx)
        
        return R
        
    except Exception as e:
        logger.error(f"Ошибка преобразования углов Эйлера в матрицу поворота: {e}")
        return np.eye(3)  # Возвращаем единичную матрицу в случае ошибки

# Основная точка входа для тестирования модуля
if __name__ == "__main__":
    # Настраиваем логирование
    setup_logger(level=logging.DEBUG)
    
    # Тестируем функции преобразования координат
    joint_angles = [45.0, 30.0, 60.0, 0.0, 0.0, 0.0]
    logger.info(f"Исходные углы: {joint_angles}")
    
    # Прямая кинематика
    position = joint_to_cartesian(joint_angles)
    logger.info(f"Позиция: {position}")
    
    # Обратная кинематика
    x, y, z, orientation = position
    calculated_angles = cartesian_to_joint(x, y, z, orientation)
    logger.info(f"Рассчитанные углы: {calculated_angles}")
    
    # Тестируем интерполяцию
    start_pos = [0.0, 0.0, 100.0]
    end_pos = [100.0, 100.0, 50.0]
    path = interpolate_positions(start_pos, end_pos, 5)
    logger.info(f"Путь: {path}")
    
    # Тестируем расчет траектории
    trajectory, time = calculate_trajectory(path)
    logger.info(f"Траектория: {trajectory}")
    logger.info(f"Общее время: {time} сек.") 