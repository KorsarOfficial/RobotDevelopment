#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Файл конфигурации для настройки параметров работы с роботом.
Содержит константы и настройки для эмуляции и реального подключения.
"""

# Режим эмуляции (True - эмуляция, False - реальный робот)
EMULATION_MODE = True

# Параметры подключения к реальному роботу
ROBOT_IP = "192.168.0.100"  # IP-адрес робота
ROBOT_PORT = 5000           # Порт подключения

# Параметры движения
MAX_SPEED = 50.0            # Максимальная скорость движения (единиц в секунду)
MAX_ACCELERATION = 10.0     # Максимальное ускорение (единиц в секунду²)

# Параметры гриппера
GRIPPER_OPEN_POSITION = 0.0      # Позиция открытого гриппера
GRIPPER_CLOSED_POSITION = 100.0  # Позиция закрытого гриппера

# Ограничения рабочей зоны
WORKSPACE_LIMITS = {
    "x_min": -300.0,
    "x_max": 300.0,
    "y_min": -300.0,
    "y_max": 300.0,
    "z_min": 0.0,
    "z_max": 300.0
}

# Ограничения углов сервоприводов (в градусах)
JOINT_LIMITS = [
    {"min": -180.0, "max": 180.0},  # Основание
    {"min": -90.0, "max": 90.0},    # Плечо
    {"min": -90.0, "max": 90.0},    # Локоть
    {"min": -180.0, "max": 180.0},  # Запястье (наклон)
    {"min": -90.0, "max": 90.0},    # Запястье (поворот)
    {"min": -180.0, "max": 180.0}   # Запястье (вращение)
]

# Домашняя позиция (в декартовых координатах)
HOME_POSITION = {
    "position": [0.0, 0.0, 150.0],
    "angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}

# Настройки логирования
LOG_LEVEL = "INFO"  # Уровень логирования (DEBUG, INFO, WARNING, ERROR, CRITICAL)
LOG_FILE = "robot.log"  # Файл для записи логов

# Пути к файлам SDK
SDK_PATH = "C:/Program Files/Robot/SDK"  # Путь к директории SDK
SDK_DLL = "risdk.dll"  # Имя DLL с SDK

# Предустановленные позиции для демонстрации
PRESETS = [
    {"name": "Исходная", "position": [0.0, 0.0, 150.0], "angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {"name": "Позиция 1", "position": [100.0, 0.0, 50.0], "angles": [0.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
    {"name": "Позиция 2", "position": [100.0, 100.0, 50.0], "angles": [45.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
    {"name": "Позиция 3", "position": [0.0, 100.0, 50.0], "angles": [90.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
    {"name": "Позиция 4", "position": [-100.0, 0.0, 50.0], "angles": [135.0, 15.0, 30.0, 0.0, 0.0, 0.0]}
]

# Скрипты автоматических последовательностей
AUTO_SCRIPTS = {
    "simple_demo": [
        {"action": "move_to_position", "params": {"x": 100.0, "y": 0.0, "z": 50.0}},
        {"action": "gripper", "params": {"close": True}},
        {"action": "move_to_position", "params": {"x": 100.0, "y": 0.0, "z": 150.0}},
        {"action": "move_to_position", "params": {"x": -100.0, "y": 0.0, "z": 150.0}},
        {"action": "move_to_position", "params": {"x": -100.0, "y": 0.0, "z": 50.0}},
        {"action": "gripper", "params": {"close": False}},
        {"action": "home_position", "params": {}}
    ],
    "pick_and_place": [
        {"action": "move_to_position", "params": {"x": 150.0, "y": 0.0, "z": 50.0}},
        {"action": "gripper", "params": {"close": True}},
        {"action": "move_to_position", "params": {"x": 150.0, "y": 0.0, "z": 100.0}},
        {"action": "move_to_position", "params": {"x": -150.0, "y": 0.0, "z": 100.0}},
        {"action": "move_to_position", "params": {"x": -150.0, "y": 0.0, "z": 50.0}},
        {"action": "gripper", "params": {"close": False}},
        {"action": "move_to_position", "params": {"x": -150.0, "y": 0.0, "z": 100.0}},
        {"action": "home_position", "params": {}}
    ]
} 