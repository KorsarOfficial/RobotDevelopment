#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Клиент для взаимодействия с API робота.
Предоставляет программный интерфейс для управления роботом-манипулятором.
Использует RISDK через ctypes или эмуляцию в тестовом режиме.
"""

import time
import sys
import os
import logging
import random
import threading
import numpy as np
from typing import List, Tuple, Dict, Optional, Union, Any
import config


class RobotClient:
    """
    Клиент для взаимодействия с роботом-манипулятором.
    Обеспечивает доступ к API робота через RISDK или эмуляцию.
    """
    
    def __init__(self):
        """Инициализация клиента."""
        # Флаг подключения
        self._connected = False
        
        # Параметры работы
        self._emulation_mode = config.EMULATION_MODE
        self._api_loaded = False
        
        # Текущее состояние робота (для эмуляции)
        self._position = [0.0, 0.0, 0.0]  # X, Y, Z
        self._angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Углы для 6 сервоприводов
        self._gripper_state = False  # False - открыт, True - закрыт
        self._is_moving = False
        self._auto_running = False
        
        # Предопределенные позиции для автоматического режима
        self._presets = [
            {"position": [100.0, 0.0, 50.0], "angles": [0.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
            {"position": [100.0, 100.0, 50.0], "angles": [45.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
            {"position": [0.0, 100.0, 50.0], "angles": [90.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
            {"position": [-100.0, 0.0, 50.0], "angles": [135.0, 15.0, 30.0, 0.0, 0.0, 0.0]},
            {"position": [0.0, 0.0, 150.0], "angles": [0.0, 30.0, 60.0, 0.0, 0.0, 0.0]},
        ]
        
        # Инициализация SDK
        self._init_api()
    
    def _init_api(self) -> bool:
        """
        Инициализация API робота.
        
        Returns:
            bool: Успешность инициализации.
        """
        if self._emulation_mode:
            logging.info("Инициализация в режиме эмуляции")
            self._api_loaded = True
            return True
        
        try:
            # Здесь должна быть реальная инициализация RISDK
            # Например:
            # self._sdk = ctypes.CDLL('risdk.dll')
            # init_result = self._sdk.Initialize()
            # if init_result != 0:
            #     logging.error(f"Ошибка инициализации SDK: {init_result}")
            #     return False
            
            # В реальном коде нужно заменить этот блок на правильную инициализацию
            logging.warning("Реальная инициализация SDK не реализована, используется эмуляция")
            self._emulation_mode = True
            self._api_loaded = True
            return True
            
        except Exception as e:
            logging.error(f"Ошибка при инициализации API робота: {e}")
            self._api_loaded = False
            return False
    
    def connect(self) -> bool:
        """
        Подключение к роботу.
        
        Returns:
            bool: Успешность подключения.
        """
        if not self._api_loaded:
            if not self._init_api():
                logging.error("Невозможно подключиться, API не инициализирован")
                return False
        
        if self._connected:
            logging.warning("Уже подключено")
            return True
        
        try:
            if self._emulation_mode:
                # Эмуляция подключения
                time.sleep(1.0)  # Имитация задержки подключения
                self._connected = True
                logging.info("Эмуляция: подключение успешно")
                return True
            else:
                # Реальное подключение
                # Например:
                # connect_result = self._sdk.Connect(config.ROBOT_IP.encode(), config.ROBOT_PORT)
                # if connect_result != 0:
                #     logging.error(f"Ошибка подключения: {connect_result}")
                #     return False
                # self._connected = True
                # return True
                
                # В реальном коде нужно заменить этот блок на правильное подключение
                logging.warning("Реальное подключение не реализовано, используется эмуляция")
                self._emulation_mode = True
                time.sleep(1.0)
                self._connected = True
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при подключении к роботу: {e}")
            return False
    
    def disconnect(self) -> bool:
        """
        Отключение от робота.
        
        Returns:
            bool: Успешность отключения.
        """
        if not self._connected:
            logging.warning("Уже отключено")
            return True
        
        try:
            # Остановка любых активных движений
            if self._is_moving or self._auto_running:
                self.stop_auto_mode()
                self._is_moving = False
            
            if self._emulation_mode:
                # Эмуляция отключения
                time.sleep(0.5)  # Имитация задержки отключения
                self._connected = False
                logging.info("Эмуляция: отключение успешно")
                return True
            else:
                # Реальное отключение
                # Например:
                # disconnect_result = self._sdk.Disconnect()
                # if disconnect_result != 0:
                #     logging.error(f"Ошибка отключения: {disconnect_result}")
                #     return False
                # self._connected = False
                # return True
                
                # В реальном коде нужно заменить этот блок на правильное отключение
                logging.warning("Реальное отключение не реализовано, используется эмуляция")
                time.sleep(0.5)
                self._connected = False
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при отключении от робота: {e}")
            return False
    
    def is_connected(self) -> bool:
        """
        Проверка состояния подключения.
        
        Returns:
            bool: True, если подключено, иначе False.
        """
        return self._connected
    
    def get_current_position(self) -> List[float]:
        """
        Получение текущей позиции робота в декартовых координатах.
        
        Returns:
            List[float]: Список [X, Y, Z] текущей позиции.
        """
        if not self._connected:
            logging.warning("Робот не подключен")
            return [0.0, 0.0, 0.0]
        
        if self._emulation_mode:
            # В режиме эмуляции возвращаем сохраненную позицию
            return self._position.copy()
        else:
            # Реальное получение позиции
            # Например:
            # pos_x = ctypes.c_double()
            # pos_y = ctypes.c_double()
            # pos_z = ctypes.c_double()
            # result = self._sdk.GetPosition(ctypes.byref(pos_x), ctypes.byref(pos_y), ctypes.byref(pos_z))
            # if result != 0:
            #     logging.error(f"Ошибка получения позиции: {result}")
            #     return [0.0, 0.0, 0.0]
            # return [pos_x.value, pos_y.value, pos_z.value]
            
            # В реальном коде нужно заменить этот блок на правильное получение позиции
            return self._position.copy()
    
    def get_current_angles(self) -> List[float]:
        """
        Получение текущих углов сервоприводов.
        
        Returns:
            List[float]: Список углов всех сервоприводов робота.
        """
        if not self._connected:
            logging.warning("Робот не подключен")
            return [0.0] * 6
        
        if self._emulation_mode:
            # В режиме эмуляции возвращаем сохраненные углы
            return self._angles.copy()
        else:
            # Реальное получение углов
            # Например:
            # angles = (ctypes.c_double * 6)()
            # result = self._sdk.GetJointAngles(angles)
            # if result != 0:
            #     logging.error(f"Ошибка получения углов: {result}")
            #     return [0.0] * 6
            # return [angles[i] for i in range(6)]
            
            # В реальном коде нужно заменить этот блок на правильное получение углов
            return self._angles.copy()
    
    def move_to_position(self, x: float, y: float, z: float) -> bool:
        """
        Перемещение робота в указанную позицию (декартовы координаты).
        
        Args:
            x: Координата X.
            y: Координата Y.
            z: Координата Z.
            
        Returns:
            bool: Успешность начала перемещения.
        """
        if not self._connected:
            logging.error("Робот не подключен")
            return False
        
        if self._auto_running:
            logging.error("Невозможно выполнить перемещение в автоматическом режиме")
            return False
        
        try:
            if self._emulation_mode:
                # Эмуляция перемещения
                self._is_moving = True
                
                # Запуск эмуляции движения в отдельном потоке
                threading.Thread(
                    target=self._emulate_movement_to_position,
                    args=(x, y, z),
                    daemon=True
                ).start()
                
                return True
            else:
                # Реальное перемещение
                # Например:
                # result = self._sdk.MoveToPosition(
                #     ctypes.c_double(x),
                #     ctypes.c_double(y),
                #     ctypes.c_double(z)
                # )
                # if result != 0:
                #     logging.error(f"Ошибка при перемещении: {result}")
                #     return False
                # return True
                
                # В реальном коде нужно заменить этот блок на правильное перемещение
                self._is_moving = True
                threading.Thread(
                    target=self._emulate_movement_to_position,
                    args=(x, y, z),
                    daemon=True
                ).start()
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при перемещении робота: {e}")
            return False
    
    def _emulate_movement_to_position(self, target_x: float, target_y: float, target_z: float):
        """
        Эмуляция движения к заданной позиции.
        
        Args:
            target_x: Целевая координата X.
            target_y: Целевая координата Y.
            target_z: Целевая координата Z.
        """
        # Сохраняем начальную позицию
        start_pos = self._position.copy()
        
        # Расстояние до цели
        distance = np.sqrt(
            (target_x - start_pos[0])**2 + 
            (target_y - start_pos[1])**2 + 
            (target_z - start_pos[2])**2
        )
        
        # Скорость движения (условных единиц в секунду)
        speed = 50.0  # Настройка скорости эмуляции
        
        # Время, необходимое для завершения движения
        total_time = distance / speed
        
        # Минимальное время движения
        if total_time < 0.5:
            total_time = 0.5
        
        # Время начала движения
        start_time = time.time()
        
        # Эмуляция движения
        while self._is_moving and self._connected:
            # Текущее время
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Прогресс движения (от 0 до 1)
            progress = min(elapsed / total_time, 1.0)
            
            # Обновление текущей позиции
            self._position[0] = start_pos[0] + (target_x - start_pos[0]) * progress
            self._position[1] = start_pos[1] + (target_y - start_pos[1]) * progress
            self._position[2] = start_pos[2] + (target_z - start_pos[2]) * progress
            
            # Обновление углов (упрощенная эмуляция)
            # В реальности нужно было бы использовать обратную кинематику
            self._update_angles_from_position()
            
            # Если движение завершено
            if progress >= 1.0:
                self._is_moving = False
                break
            
            # Пауза для снижения нагрузки на CPU
            time.sleep(0.05)
    
    def _update_angles_from_position(self):
        """
        Обновление углов на основе текущей позиции (упрощенная эмуляция).
        В реальности здесь должен быть расчет обратной кинематики.
        """
        # Очень упрощенная эмуляция
        x, y, z = self._position
        
        # Угол поворота основания (в плоскости XY)
        if x == 0 and y == 0:
            angle_base = 0
        else:
            angle_base = np.arctan2(y, x) * 180 / np.pi
        
        # Расстояние от основания до конечной точки
        distance = np.sqrt(x**2 + y**2 + z**2)
        
        # Очень приблизительная эмуляция углов
        angle_shoulder = min(45, distance / 5)
        angle_elbow = min(90, distance / 3)
        
        # Обновление углов
        self._angles[0] = angle_base  # Основание
        self._angles[1] = angle_shoulder  # Плечо
        self._angles[2] = angle_elbow  # Локоть
        self._angles[3] = 0  # Запястье (наклон)
        self._angles[4] = 0  # Запястье (поворот)
        self._angles[5] = 0  # Запястье (вращение)
    
    def move_to_angles(self, angles: List[float]) -> bool:
        """
        Перемещение сервоприводов в указанные углы.
        
        Args:
            angles: Список углов для всех сервоприводов робота.
            
        Returns:
            bool: Успешность начала перемещения.
        """
        if not self._connected:
            logging.error("Робот не подключен")
            return False
        
        if self._auto_running:
            logging.error("Невозможно выполнить перемещение в автоматическом режиме")
            return False
        
        if len(angles) != 6:
            logging.error(f"Неверное количество углов: {len(angles)}, ожидается 6")
            return False
        
        try:
            if self._emulation_mode:
                # Эмуляция перемещения
                self._is_moving = True
                
                # Запуск эмуляции движения в отдельном потоке
                threading.Thread(
                    target=self._emulate_movement_to_angles,
                    args=(angles,),
                    daemon=True
                ).start()
                
                return True
            else:
                # Реальное перемещение
                # Например:
                # angles_array = (ctypes.c_double * 6)(*angles)
                # result = self._sdk.MoveToAngles(angles_array)
                # if result != 0:
                #     logging.error(f"Ошибка при перемещении: {result}")
                #     return False
                # return True
                
                # В реальном коде нужно заменить этот блок на правильное перемещение
                self._is_moving = True
                threading.Thread(
                    target=self._emulate_movement_to_angles,
                    args=(angles,),
                    daemon=True
                ).start()
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при перемещении робота: {e}")
            return False
    
    def _emulate_movement_to_angles(self, target_angles: List[float]):
        """
        Эмуляция движения к заданным углам.
        
        Args:
            target_angles: Целевые углы сервоприводов.
        """
        # Сохраняем начальные углы
        start_angles = self._angles.copy()
        
        # Максимальное изменение угла
        max_angle_diff = 0
        for i in range(len(start_angles)):
            diff = abs(target_angles[i] - start_angles[i])
            if diff > max_angle_diff:
                max_angle_diff = diff
        
        # Скорость движения (градусов в секунду)
        speed = 30.0  # Настройка скорости эмуляции
        
        # Время, необходимое для завершения движения
        total_time = max_angle_diff / speed
        
        # Минимальное время движения
        if total_time < 0.5:
            total_time = 0.5
        
        # Время начала движения
        start_time = time.time()
        
        # Эмуляция движения
        while self._is_moving and self._connected:
            # Текущее время
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Прогресс движения (от 0 до 1)
            progress = min(elapsed / total_time, 1.0)
            
            # Обновление текущих углов
            for i in range(len(self._angles)):
                self._angles[i] = start_angles[i] + (target_angles[i] - start_angles[i]) * progress
            
            # Обновление позиции на основе углов
            self._update_position_from_angles()
            
            # Если движение завершено
            if progress >= 1.0:
                self._is_moving = False
                break
            
            # Пауза для снижения нагрузки на CPU
            time.sleep(0.05)
    
    def _update_position_from_angles(self):
        """
        Обновление позиции на основе текущих углов (упрощенная эмуляция).
        В реальности здесь должен быть расчет прямой кинематики.
        """
        # Очень упрощенная эмуляция
        angle_base = self._angles[0]  # Основание
        angle_shoulder = self._angles[1]  # Плечо
        angle_elbow = self._angles[2]  # Локоть
        
        # Расстояние от основания (пропорционально углам)
        distance = (angle_shoulder * 5 + angle_elbow * 3) / 2
        
        # Позиция в полярных координатах (от основания)
        angle_rad = angle_base * np.pi / 180
        
        # Преобразование в декартовы координаты
        self._position[0] = distance * np.cos(angle_rad)
        self._position[1] = distance * np.sin(angle_rad)
        self._position[2] = distance / 2  # Упрощенная зависимость Z от расстояния
    
    def home_position(self) -> bool:
        """
        Перемещение робота в домашнюю позицию.
        
        Returns:
            bool: Успешность начала перемещения.
        """
        # Домашняя позиция (зависит от робота)
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Перемещение в домашнюю позицию
        return self.move_to_angles(home_angles)
    
    def set_gripper(self, close: bool) -> bool:
        """
        Управление гриппером.
        
        Args:
            close: True для захвата, False для разжима.
            
        Returns:
            bool: Успешность операции.
        """
        if not self._connected:
            logging.error("Робот не подключен")
            return False
        
        try:
            if self._emulation_mode:
                # Эмуляция работы гриппера
                time.sleep(0.5)  # Имитация задержки при работе гриппера
                self._gripper_state = close
                logging.info(f"Эмуляция: гриппер {'закрыт' if close else 'открыт'}")
                return True
            else:
                # Реальное управление гриппером
                # Например:
                # result = self._sdk.SetGripper(1 if close else 0)
                # if result != 0:
                #     logging.error(f"Ошибка при управлении гриппером: {result}")
                #     return False
                # self._gripper_state = close
                # return True
                
                # В реальном коде нужно заменить этот блок на правильное управление
                time.sleep(0.5)
                self._gripper_state = close
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при управлении гриппером: {e}")
            return False
    
    def is_gripper_closed(self) -> bool:
        """
        Получение состояния гриппера.
        
        Returns:
            bool: True, если гриппер закрыт, иначе False.
        """
        if not self._connected:
            logging.warning("Робот не подключен")
            return False
        
        if self._emulation_mode:
            return self._gripper_state
        else:
            # Реальное получение состояния гриппера
            # Например:
            # state = ctypes.c_int()
            # result = self._sdk.GetGripperState(ctypes.byref(state))
            # if result != 0:
            #     logging.error(f"Ошибка при получении состояния гриппера: {result}")
            #     return False
            # return state.value != 0
            
            # В реальном коде нужно заменить этот блок на правильное получение состояния
            return self._gripper_state
    
    def emergency_stop(self) -> bool:
        """
        Аварийная остановка робота.
        
        Returns:
            bool: Успешность операции.
        """
        if not self._connected:
            logging.error("Робот не подключен")
            return False
        
        try:
            # Остановка любых движений
            self._is_moving = False
            self._auto_running = False
            
            if self._emulation_mode:
                # Эмуляция аварийной остановки
                logging.info("Эмуляция: аварийная остановка выполнена")
                return True
            else:
                # Реальная аварийная остановка
                # Например:
                # result = self._sdk.EmergencyStop()
                # if result != 0:
                #     logging.error(f"Ошибка при аварийной остановке: {result}")
                #     return False
                # return True
                
                # В реальном коде нужно заменить этот блок на правильную остановку
                return True
                
        except Exception as e:
            logging.error(f"Ошибка при аварийной остановке: {e}")
            return False
    
    def goto_preset(self, preset_index: int) -> bool:
        """
        Перемещение к предустановленной позиции.
        
        Args:
            preset_index: Индекс предустановленной позиции.
            
        Returns:
            bool: Успешность начала перемещения.
        """
        if preset_index < 0 or preset_index >= len(self._presets):
            logging.error(f"Неверный индекс предустановленной позиции: {preset_index}")
            return False
        
        # Получение предустановленной позиции
        preset = self._presets[preset_index]
        
        # Перемещение к позиции
        return self.move_to_position(*preset["position"])
    
    def start_auto_mode(self, cycle: bool = False) -> bool:
        """
        Запуск автоматического режима.
        
        Args:
            cycle: True для циклического режима, False для одиночного прохода.
            
        Returns:
            bool: Успешность запуска.
        """
        if not self._connected:
            logging.error("Робот не подключен")
            return False
        
        if self._auto_running:
            logging.warning("Автоматический режим уже запущен")
            return True
        
        try:
            # Установка флага автоматического режима
            self._auto_running = True
            
            # Запуск автоматического режима в отдельном потоке
            threading.Thread(
                target=self._run_auto_mode,
                args=(cycle,),
                daemon=True
            ).start()
            
            return True
            
        except Exception as e:
            logging.error(f"Ошибка при запуске автоматического режима: {e}")
            self._auto_running = False
            return False
    
    def _run_auto_mode(self, cycle: bool):
        """
        Выполнение автоматического режима.
        
        Args:
            cycle: True для циклического режима, False для одиночного прохода.
        """
        try:
            # Цикл автоматического режима
            while self._auto_running and self._connected:
                # Проход по всем предустановленным позициям
                for i, preset in enumerate(self._presets):
                    # Проверка, что автоматический режим все еще активен
                    if not self._auto_running or not self._connected:
                        break
                    
                    # Перемещение к позиции
                    logging.info(f"Автоматический режим: переход к позиции {i+1}")
                    
                    # Сохраняем текущие значения
                    self._is_moving = True
                    position = preset["position"]
                    
                    # Эмуляция движения в отдельном потоке
                    self._emulate_movement_to_position(*position)
                    
                    # Ожидание завершения движения
                    while self._is_moving and self._auto_running and self._connected:
                        time.sleep(0.1)
                    
                    # Пауза между позициями
                    if self._auto_running and self._connected:
                        time.sleep(1.0)
                
                # Если не циклический режим, то выходим после одного прохода
                if not cycle:
                    break
            
            # Сброс флага автоматического режима
            self._auto_running = False
            logging.info("Автоматический режим завершен")
            
        except Exception as e:
            logging.error(f"Ошибка в автоматическом режиме: {e}")
            self._auto_running = False
    
    def stop_auto_mode(self) -> bool:
        """
        Остановка автоматического режима.
        
        Returns:
            bool: Успешность остановки.
        """
        if not self._auto_running:
            logging.warning("Автоматический режим не запущен")
            return True
        
        try:
            # Сброс флагов
            self._auto_running = False
            self._is_moving = False
            
            # Ожидание полной остановки
            time.sleep(0.5)
            
            logging.info("Автоматический режим остановлен")
            return True
            
        except Exception as e:
            logging.error(f"Ошибка при остановке автоматического режима: {e}")
            return False


# Пример использования
if __name__ == "__main__":
    # Настройка логирования
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Создание клиента робота
    robot = RobotClient()
    
    # Подключение к роботу
    if robot.connect():
        logging.info("Подключение успешно")
        
        # Перемещение в домашнюю позицию
        if robot.home_position():
            logging.info("Перемещение в домашнюю позицию")
        
        # Ожидание завершения движения
        time.sleep(2)
        
        # Перемещение к заданной позиции
        if robot.move_to_position(100, 100, 50):
            logging.info("Перемещение к позиции X=100, Y=100, Z=50")
        
        # Ожидание завершения движения
        time.sleep(3)
        
        # Управление гриппером
        if robot.set_gripper(True):
            logging.info("Гриппер закрыт")
        
        time.sleep(1)
        
        if robot.set_gripper(False):
            logging.info("Гриппер открыт")
        
        # Отключение от робота
        if robot.disconnect():
            logging.info("Отключение успешно")
    else:
        logging.error("Ошибка подключения") 