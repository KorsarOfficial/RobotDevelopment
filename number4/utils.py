#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Вспомогательные функции для работы с роботом
"""

import os
import sys
import time
import logging
import numpy as np
from pathlib import Path

# Импортируем конфигурацию
try:
    from config import *
except ImportError:
    # Если конфигурация не найдена в текущей директории
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from number4.config import *


def setup_logger(log_level=LOG_LEVEL, log_file=LOG_FILE):
    """
    Настройка логирования
    
    Args:
        log_level (str): Уровень логирования
        log_file (str): Путь к файлу логов
    
    Returns:
        logging.Logger: Настроенный логгер
    """
    # Создаем директорию для логов, если ее нет
    log_dir = os.path.dirname(log_file)
    if log_dir and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Настраиваем логгер
    logger = logging.getLogger("robot_rotation")
    logger.setLevel(getattr(logging, log_level))
    
    # Обработчик для файла
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setLevel(getattr(logging, log_level))
    
    # Обработчик для консоли
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, log_level))
    
    # Формат вывода
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Добавляем обработчики к логгеру
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


def generate_rotation_angles():
    """
    Генерация последовательности углов для вращения вокруг оси X
    
    Returns:
        list: Список углов поворота
    """
    min_angle = X_ROTATION_PARAMS["min_angle"]
    max_angle = X_ROTATION_PARAMS["max_angle"]
    steps = X_ROTATION_PARAMS["steps"]
    
    # Генерация последовательности углов от min_angle до max_angle и обратно
    angles_forward = np.linspace(min_angle, max_angle, steps).tolist()
    angles_backward = np.linspace(max_angle, min_angle, steps).tolist()
    
    return angles_forward + angles_backward


def format_position(position):
    """
    Форматирование позиции для вывода
    
    Args:
        position (dict): Словарь с координатами
    
    Returns:
        str: Отформатированная строка с координатами
    """
    return f"X: {position['x']:.2f}, Y: {position['y']:.2f}, Z: {position['z']:.2f}, " \
           f"Roll: {position['roll']:.2f}, Pitch: {position['pitch']:.2f}, Yaw: {position['yaw']:.2f}"


def interpolate_positions(start_pos, end_pos, steps=10):
    """
    Линейная интерполяция между двумя позициями
    
    Args:
        start_pos (dict): Начальная позиция
        end_pos (dict): Конечная позиция
        steps (int): Количество шагов

    Returns:
        list: Список позиций для плавного перехода
    """
    positions = []
    
    for i in range(steps + 1):
        t = i / steps
        pos = {}
        
        for key in start_pos:
            pos[key] = start_pos[key] + t * (end_pos[key] - start_pos[key])
        
        positions.append(pos)
    
    return positions


def validate_position(position):
    """
    Проверка позиции на допустимость
    
    Args:
        position (dict): Позиция для проверки
        
    Returns:
        bool: True, если позиция допустима
    """
    # Проверяем наличие всех необходимых ключей
    required_keys = ["x", "y", "z", "roll", "pitch", "yaw"]
    for key in required_keys:
        if key not in position:
            return False
    
    return True 