#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Контроллер для управления роботом и выполнения вращения вокруг оси X
"""

import time
import sys
import os
from ctypes import *
import logging

# Импортируем конфигурацию и утилиты
try:
    from config import *
    from utils import format_position, generate_rotation_angles
except ImportError:
    # Если файлы не найдены в текущей директории
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from number4.config import *
    from number4.utils import format_position, generate_rotation_angles

class RobotController:
    """Класс для управления роботом."""
    
    def __init__(self, emulation=EMULATION_MODE):
        """
        Инициализация контроллера робота.
        
        Args:
            emulation (bool): Включить режим эмуляции.
        """
        self.emulation = emulation
        self.connected = False
        self.logger = logging.getLogger("robot_rotation.RobotController")
        self.logger.info("Инициализация контроллера робота (эмуляция: %s)", emulation)
        
        # Подключение библиотеки RISDK
        try:
            # Подключение библиотеки ri_sdk
            self.lib = cdll.LoadLibrary("./librisdk.dll")
            
            # Объявление аргументов вызываемых функций
            self._setup_function_argtypes()
            
            # Объявление переменных для хранения дескрипторов компонентов
            self.errTextC = create_string_buffer(1000)  # Переменная, в которую будет записываться текст ошибки
            self.errCode = c_int()  # Переменная, в которую будет записываться код ошибки
            
            # Дескрипторы компонентов
            self.i2c = c_int()  # Переменная, в которой будет храниться дескриптор на объект i2c адаптера
            self.pwm = c_int()  # Переменная, в которой будет храниться дескриптор на объект pwm модулятора
            self.servo_base = c_int()  # Дескриптор сервопривода основания (поворачивает вокруг оси Z)
            self.servo_shoulder = c_int()  # Дескриптор сервопривода плеча (поворачивает вокруг оси X)
            self.servo_elbow = c_int()  # Дескриптор сервопривода локтя
            self.servo_wrist = c_int()  # Дескриптор сервопривода запястья
            self.servo_gripper = c_int()  # Дескриптор сервопривода захвата
            
            # Инициализация компонентов
            self._init_components()
            
            self.connected = True
            self.logger.info("Робот успешно инициализирован")
            
        except Exception as e:
            self.logger.error("Ошибка инициализации RISDK: %s", str(e))
            raise
    
    def _setup_function_argtypes(self):
        """Настройка типов аргументов для функций RISDK."""
        # Инициализация SDK
        self.lib.RI_SDK_InitSDK.argtypes = [c_int, c_char_p]
        
        # Создание компонентов
        self.lib.RI_SDK_CreateModelComponent.argtypes = [c_char_p, c_char_p, c_char_p, POINTER(c_int), c_char_p]
        
        # Связывание компонентов
        self.lib.RI_SDK_LinkPWMToController.argtypes = [c_int, c_int, c_uint8, c_char_p]
        self.lib.RI_SDK_LinkServodriveToController.argtypes = [c_int, c_int, c_int, c_char_p]
        
        # Управление сервоприводами
        self.lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
        self.lib.RI_SDK_exec_ServoDrive_TurnByPulse.argtypes = [c_int, c_int, c_char_p]
        self.lib.RI_SDK_exec_ServoDrive_GetCurrentAngle.argtypes = [c_int, POINTER(c_int), c_char_p]
        
        # Управление ШИМ
        self.lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
        self.lib.RI_SDK_sigmod_PWM_ResetPort.argtypes = [c_int, c_int, c_char_p]
        
        # Удаление компонентов
        self.lib.RI_SDK_DestroyComponent.argtypes = [c_int, c_char_p]
        self.lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]
    
    def _init_components(self):
        """Инициализация компонентов робота."""
        self.logger.info("Инициализация компонентов робота")
        
        # Инициализация RI SDK
        self.errCode = self.lib.RI_SDK_InitSDK(2, self.errTextC)
        if self.errCode != 0:
            self.logger.error("Ошибка инициализации SDK: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Если режим эмуляции включен, то пропускаем инициализацию физических компонентов
        if self.emulation:
            self.logger.info("Работа в режиме эмуляции, физические компоненты не инициализируются")
            return
        
        # Инициализация i2c адаптера для RM 001 (cp2112)
        self.errCode = self.lib.RI_SDK_CreateModelComponent(
            "connector".encode(), 
            "i2c_adapter".encode(), 
            "cp2112".encode(),  # Используем модель cp2112 для RM 001
            self.i2c, 
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка инициализации i2c адаптера: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Инициализация ШИМ модулятора
        self.errCode = self.lib.RI_SDK_CreateModelComponent(
            "connector".encode(), 
            "pwm".encode(), 
            "pca9685".encode(),  # Используем модель pca9685 для RM 001
            self.pwm, 
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка инициализации ШИМ модулятора: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Связывание ШИМ с i2c адаптером
        self.errCode = self.lib.RI_SDK_LinkPWMToController(
            self.pwm, 
            self.i2c, 
            c_uint8(0x40),  # Адрес ШИМ модулятора на шине i2c
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка связывания ШИМ с i2c адаптером: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Инициализация сервопривода основания (ось Z)
        self.errCode = self.lib.RI_SDK_CreateModelComponent(
            "executor".encode(), 
            "servodrive".encode(), 
            "mg90s".encode(),  # Используем модель mg90s для сервоприводов RM 001
            self.servo_base, 
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка инициализации сервопривода основания: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Связывание сервопривода основания с ШИМ
        self.errCode = self.lib.RI_SDK_LinkServodriveToController(
            self.servo_base, 
            self.pwm, 
            0,  # Порт 0 для сервопривода основания
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка связывания сервопривода основания с ШИМ: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Инициализация сервопривода плеча (ось X)
        self.errCode = self.lib.RI_SDK_CreateModelComponent(
            "executor".encode(), 
            "servodrive".encode(), 
            "mg90s".encode(), 
            self.servo_shoulder, 
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка инициализации сервопривода плеча: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Связывание сервопривода плеча с ШИМ
        self.errCode = self.lib.RI_SDK_LinkServodriveToController(
            self.servo_shoulder, 
            self.pwm, 
            1,  # Порт 1 для сервопривода плеча
            self.errTextC
        )
        if self.errCode != 0:
            self.logger.error("Ошибка связывания сервопривода плеча с ШИМ: %s", self.errTextC.raw.decode())
            sys.exit(2)
        
        # Инициализация остальных сервоприводов...
        # В примере нам нужны только сервоприводы основания и плеча для вращения по осям Z и X
    
    def __del__(self):
        """Деструктор для освобождения ресурсов."""
        self.disconnect()
    
    def disconnect(self):
        """Отключение от робота и освобождение ресурсов."""
        if hasattr(self, 'connected') and self.connected:
            self.logger.info("Отключение от робота")
            
            if not self.emulation:
                try:
                    # Сброс сигналов со всех портов ШИМ
                    self.errCode = self.lib.RI_SDK_sigmod_PWM_ResetAll(self.pwm, self.errTextC)
                    if self.errCode != 0:
                        self.logger.error("Ошибка сброса сигналов ШИМ: %s", self.errTextC.raw.decode())
                    
                    # Удаление компонентов в порядке, обратном созданию
                    if hasattr(self, 'servo_gripper'):
                        self.lib.RI_SDK_DestroyComponent(self.servo_gripper, self.errTextC)
                    
                    if hasattr(self, 'servo_wrist'):
                        self.lib.RI_SDK_DestroyComponent(self.servo_wrist, self.errTextC)
                    
                    if hasattr(self, 'servo_elbow'):
                        self.lib.RI_SDK_DestroyComponent(self.servo_elbow, self.errTextC)
                    
                    if hasattr(self, 'servo_shoulder'):
                        self.lib.RI_SDK_DestroyComponent(self.servo_shoulder, self.errTextC)
                    
                    if hasattr(self, 'servo_base'):
                        self.lib.RI_SDK_DestroyComponent(self.servo_base, self.errTextC)
                    
                    if hasattr(self, 'pwm'):
                        self.lib.RI_SDK_DestroyComponent(self.pwm, self.errTextC)
                    
                    if hasattr(self, 'i2c'):
                        self.lib.RI_SDK_DestroyComponent(self.i2c, self.errTextC)
                    
                except Exception as e:
                    self.logger.error("Ошибка при удалении компонентов: %s", str(e))
            
            # Завершение работы с RI SDK
            try:
                self.lib.RI_SDK_DestroySDK(c_bool(True), self.errTextC)
            except Exception as e:
                self.logger.error("Ошибка при завершении работы с SDK: %s", str(e))
            
            self.connected = False
            self.logger.info("Робот отключен")
    
    def move_to_home(self):
        """
        Перемещение робота в домашнюю позицию.
        
        Returns:
            bool: True, если перемещение успешно.
        """
        self.logger.info("Перемещение в домашнюю позицию")
        
        if self.emulation:
            self.logger.info("Эмуляция: робот перемещен в домашнюю позицию")
            return True
        
        try:
            # Поворачиваем сервоприводы в нейтральные позиции
            # Для робота RM 001 нейтральные позиции могут отличаться
            # Обычно это 90 градусов для сервоприводов
            
            self._turn_servo(self.servo_base, 90, SERVO_SPEED)
            time.sleep(0.5)  # Пауза для стабилизации
            
            self._turn_servo(self.servo_shoulder, 90, SERVO_SPEED)
            time.sleep(0.5)  # Пауза для стабилизации
            
            # Если нужно, добавить перемещение других сервоприводов
            
            return True
            
        except Exception as e:
            self.logger.error("Ошибка при перемещении в домашнюю позицию: %s", str(e))
            return False
    
    def _turn_servo(self, servo_descriptor, angle, speed):
        """
        Поворот сервопривода на заданный угол.
        
        Args:
            servo_descriptor (c_int): Дескриптор сервопривода.
            angle (int): Угол поворота в градусах.
            speed (int): Скорость поворота.
        
        Returns:
            bool: True, если поворот успешен.
        """
        if self.emulation:
            self.logger.info("Эмуляция: поворот сервопривода на угол %d°", angle)
            return True
        
        self.errCode = self.lib.RI_SDK_exec_ServoDrive_Turn(
            servo_descriptor, 
            angle, 
            speed, 
            c_bool(False),  # блокирующий режим
            self.errTextC
        )
        
        if self.errCode != 0:
            self.logger.error("Ошибка поворота сервопривода: %s", self.errTextC.raw.decode())
            return False
        
        return True
    
    def rotate_x_axis(self, angle):
        """
        Поворот робота вокруг оси X (сервопривод плеча).
        
        Args:
            angle (float): Угол поворота в градусах.
        
        Returns:
            bool: True, если поворот успешен.
        """
        if not self.connected:
            self.logger.error("Робот не подключен")
            return False
        
        self.logger.info("Поворот вокруг оси X на угол: %f градусов", angle)
        
        if self.emulation:
            # В режиме эмуляции просто ждем
            time.sleep(1.0)
            self.logger.info("Эмуляция: робот повернулся вокруг оси X")
            return True
        
        try:
            # Корректируем угол для сервопривода плеча
            # В зависимости от механики RM 001, может потребоваться смещение угла
            adjusted_angle = angle + 90  # Обычно 90° это центральная позиция сервопривода
            
            # Поворот сервопривода плеча (ось X)
            return self._turn_servo(self.servo_shoulder, adjusted_angle, SERVO_SPEED)
            
        except Exception as e:
            self.logger.error("Ошибка при повороте вокруг оси X: %s", str(e))
            return False
    
    def perform_x_rotation_sequence(self, cycles=1):
        """
        Выполнение последовательности поворотов вокруг оси X.
        
        Args:
            cycles (int): Количество циклов поворота.
        
        Returns:
            bool: True, если последовательность успешно выполнена.
        """
        self.logger.info("Начало последовательности поворотов вокруг оси X (%d циклов)", cycles)
        
        # Сначала перемещаемся в домашнюю позицию
        if not self.move_to_home():
            self.logger.error("Не удалось переместиться в домашнюю позицию")
            return False
        
        # Получаем последовательность углов для поворота
        angles = generate_rotation_angles()
        
        try:
            # Выполняем заданное количество циклов
            for cycle in range(cycles):
                self.logger.info("Цикл поворота %d/%d", cycle + 1, cycles)
                
                for angle in angles:
                    # Поворот на текущий угол
                    if not self.rotate_x_axis(angle):
                        self.logger.error("Ошибка при повороте на угол %f", angle)
                        return False
                    
                    # Пауза между поворотами
                    time.sleep(X_ROTATION_PARAMS["pause"])
            
            # Возвращаемся в домашнюю позицию
            self.move_to_home()
            
            self.logger.info("Последовательность поворотов успешно завершена")
            return True
            
        except Exception as e:
            self.logger.error("Ошибка при выполнении последовательности поворотов: %s", str(e))
            return False 