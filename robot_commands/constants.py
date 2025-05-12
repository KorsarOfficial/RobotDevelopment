#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Файл констант для скриптов управления роботом RM 001

# Подключение библиотеки
PLATFORM_WINDOWS = "Windows"
PLATFORM_LINUX = "Linux"

# Уровень логирования
LOG_LEVEL = 2

# Параметры сервоприводов
BODY_START_PULSE = 1500      # стартовая позиция тела
ARROW_R_START_PULSE = 2000   # стартовая позиция правой стрелы
ARROW_L_START_PULSE = 1000   # стартовая позиция левой стрелы
CLAW_START_PULSE = 1000      # стартовая позиция клешни
CLAW_ROTATE_START_PULSE = 1500  # стартовая позиция поворота клешни

# Параметры движения
MOVE_LEFT_ANGLE = -45        # угол поворота тела влево
MOVE_RIGHT_ANGLE = 45        # угол поворота тела вправо
MOVE_UP_ANGLE_R = -60        # угол поднятия правой стрелы
MOVE_UP_ANGLE_L = 60         # угол поднятия левой стрелы
MOVE_DOWN_ANGLE_R = -30      # угол опускания правой стрелы
MOVE_DOWN_ANGLE_L = 30       # угол опускания левой стрелы

# Параметры работы с кубиком
ARROW_R_OVER_CUBE_POSITION = -30  # позиция правой стрелы над кубиком
ARROW_L_OVER_CUBE_POSITION = -10  # позиция левой стрелы над кубиком
CLAW_UNCLENCHED_POSITION = 80     # позиция открытой клешни
ARROW_R_CUBE_POSITION = -75       # позиция правой стрелы на месте кубика
ARROW_L_CUBE_POSITION = 55        # позиция левой стрелы на месте кубика

# Скорость движения
SERVO_SPEED = 100            # скорость в градусах в секунду

# Команды интерактивного режима
CMD_LEFT = "left"            # команда движения влево
CMD_RIGHT = "right"          # команда движения вправо
CMD_UP = "up"                # команда движения вверх
CMD_DOWN = "down"            # команда движения вниз
CMD_OPEN = "open"            # команда открытия клешни
CMD_CLOSE = "close"          # команда закрытия клешни
CMD_HOME = "home"            # команда возврата в домашнюю позицию
CMD_EXIT = "exit"            # команда выхода
CMD_HELP = "help"            # команда справки 