#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
API для работы с роботом-манипулятором.
Реализует базовые команды для управления роботом и захватом.
"""

import time
import logging
import socket
import threading
import json
from typing import List, Tuple, Dict, Optional, Union, Any

from .utils import (
    cartesian_to_joint,
    joint_to_cartesian,
    validate_position,
    validate_joint_angles,
    interpolate_positions,
    calculate_trajectory
)

# Настройка логирования
logger = logging.getLogger(__name__)

class RobotAPI:
    """
    Класс для взаимодействия с роботом-манипулятором.
    Поддерживает режим эмуляции и реальное управление.
    """
    
    def __init__(self, config: Dict[str, Any]) -> None:
        """
        Инициализация API робота.
        
        Args:
            config: Словарь с параметрами конфигурации робота
        """
        self.config = config
        self.emulation_mode = config.get("EMULATION_MODE", True)
        
        # Параметры подключения
        self.robot_ip = config.get("ROBOT_IP", "192.168.0.100")
        self.robot_port = config.get("ROBOT_PORT", 5000)
        
        # Состояние робота
        self.connected = False
        self.current_position = config.get("HOME_POSITION", {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # По умолчанию все суставы в нуле
        self.gripper_position = 0.0  # Захват открыт (0.0 - открыт, 100.0 - закрыт)
        
        # Параметры движения
        self.max_speed = config.get("MAX_SPEED", 50.0)  # мм/с или градусов/с
        self.max_acceleration = config.get("MAX_ACCELERATION", 10.0)  # мм/с² или градусов/с²
        
        # Рабочая зона
        self.workspace_limits = config.get("WORKSPACE_LIMITS", {
            "x_min": -500.0, "x_max": 500.0,
            "y_min": -500.0, "y_max": 500.0,
            "z_min": 0.0, "z_max": 800.0
        })
        
        # Ограничения суставов
        self.joint_limits = config.get("JOINT_LIMITS", [
            {"min": -180.0, "max": 180.0},  # Основание
            {"min": -90.0, "max": 90.0},    # Плечо
            {"min": -180.0, "max": 180.0},  # Локоть
            {"min": -180.0, "max": 180.0},  # Запястье (вращение)
            {"min": -90.0, "max": 90.0},    # Запястье (наклон)
            {"min": -180.0, "max": 180.0}   # Запястье (поворот)
        ])
        
        # Связь с реальным роботом
        self.socket = None
        self.socket_lock = threading.Lock()
        
        # Эмуляция движения
        self.movement_thread = None
        self.stop_movement = threading.Event()
        
        logger.info(f"Инициализация API робота в режиме {'эмуляции' if self.emulation_mode else 'реального управления'}")
    
    def connect(self) -> bool:
        """
        Подключение к роботу.
        
        Returns:
            bool: True, если подключение успешно, иначе False
        """
        if self.emulation_mode:
            logger.info("Эмуляция подключения к роботу")
            self.connected = True
            return True
        
        try:
            logger.info(f"Подключение к роботу по адресу {self.robot_ip}:{self.robot_port}")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # Таймаут 5 секунд
            self.socket.connect((self.robot_ip, self.robot_port))
            
            # Проверка связи с роботом
            if self._send_command("VERSION"):
                self.connected = True
                logger.info("Успешное подключение к роботу")
                return True
            else:
                logger.error("Не удалось установить связь с роботом")
                self.socket.close()
                self.socket = None
                return False
                
        except Exception as e:
            logger.error(f"Ошибка подключения к роботу: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
    
    def disconnect(self) -> bool:
        """
        Отключение от робота.
        
        Returns:
            bool: True, если отключение успешно, иначе False
        """
        if self.emulation_mode:
            logger.info("Эмуляция отключения от робота")
            self.connected = False
            return True
            
        try:
            if self.socket:
                logger.info("Отключение от робота")
                self._send_command("DISCONNECT")
                self.socket.close()
                self.socket = None
            
            self.connected = False
            return True
            
        except Exception as e:
            logger.error(f"Ошибка отключения от робота: {e}")
            return False
    
    def _send_command(self, command: str, data: Any = None) -> Optional[Dict[str, Any]]:
        """
        Отправка команды роботу.
        
        Args:
            command: Команда для отправки
            data: Дополнительные данные для команды
            
        Returns:
            Optional[Dict[str, Any]]: Ответ робота или None в случае ошибки
        """
        if self.emulation_mode:
            # Эмуляция ответа от робота
            time.sleep(0.1)  # Имитация задержки связи
            return {"status": "ok", "command": command}
            
        if not self.socket:
            logger.error("Нет соединения с роботом")
            return None
            
        try:
            # Формируем JSON-сообщение
            message = {
                "command": command,
                "data": data
            }
            message_str = json.dumps(message) + "\n"
            
            with self.socket_lock:
                # Отправляем сообщение
                self.socket.sendall(message_str.encode('utf-8'))
                
                # Получаем ответ
                response = self.socket.recv(4096)
                response_str = response.decode('utf-8').strip()
                
                # Парсим JSON-ответ
                try:
                    response_data = json.loads(response_str)
                    return response_data
                except json.JSONDecodeError as e:
                    logger.error(f"Ошибка декодирования ответа от робота: {e}")
                    return None
                    
        except Exception as e:
            logger.error(f"Ошибка отправки команды роботу: {e}")
            return None
    
    def get_position(self) -> Dict[str, float]:
        """
        Получение текущей позиции робота.
        
        Returns:
            Dict[str, float]: Словарь с координатами и ориентацией робота
        """
        if self.emulation_mode:
            logger.debug(f"Эмуляция получения позиции: {self.current_position}")
            return self.current_position.copy()
            
        response = self._send_command("GET_POSITION")
        if response and response.get("status") == "ok":
            self.current_position = response.get("data", {})
            return self.current_position.copy()
        else:
            logger.error("Не удалось получить текущую позицию робота")
            return self.current_position.copy()
    
    def get_joint_angles(self) -> List[float]:
        """
        Получение текущих углов суставов робота.
        
        Returns:
            List[float]: Список углов суставов
        """
        if self.emulation_mode:
            logger.debug(f"Эмуляция получения углов суставов: {self.current_joint_angles}")
            return self.current_joint_angles.copy()
            
        response = self._send_command("GET_JOINT_ANGLES")
        if response and response.get("status") == "ok":
            self.current_joint_angles = response.get("data", [])
            return self.current_joint_angles.copy()
        else:
            logger.error("Не удалось получить текущие углы суставов робота")
            return self.current_joint_angles.copy()
    
    def move_to_position(self, x: float, y: float, z: float, 
                         roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, 
                         speed: float = None, wait: bool = True) -> bool:
        """
        Перемещение робота в указанную позицию.
        
        Args:
            x, y, z: Целевые координаты
            roll, pitch, yaw: Целевая ориентация инструмента
            speed: Скорость перемещения (% от максимальной)
            wait: Ожидать ли завершения перемещения
            
        Returns:
            bool: True, если перемещение выполнено успешно, иначе False
        """
        # Проверка на допустимость координат
        if not validate_position(x, y, z, self.workspace_limits):
            logger.error(f"Координаты ({x}, {y}, {z}) вне рабочей зоны")
            return False
        
        # Расчет углов суставов (обратная кинематика)
        orientation = [roll, pitch, yaw]
        joint_angles = cartesian_to_joint(x, y, z, orientation)
        
        if joint_angles is None:
            logger.error("Не удалось рассчитать углы суставов для указанной позиции")
            return False
        
        # Проверка на допустимость углов суставов
        if not validate_joint_angles(joint_angles, self.joint_limits):
            logger.error("Расчитанные углы суставов выходят за допустимые пределы")
            return False
        
        # Устанавливаем скорость перемещения
        if speed is None:
            speed = self.max_speed
        else:
            speed = min(speed, self.max_speed)  # Ограничиваем максимальной скоростью
        
        if self.emulation_mode:
            # Эмуляция перемещения
            if self.movement_thread and self.movement_thread.is_alive():
                # Если уже идет перемещение, останавливаем его
                self.stop_movement.set()
                self.movement_thread.join()
                self.stop_movement.clear()
            
            # Запускаем новое перемещение
            target_position = {
                "x": x, "y": y, "z": z,
                "roll": roll, "pitch": pitch, "yaw": yaw
            }
            
            self.movement_thread = threading.Thread(
                target=self._emulate_movement,
                args=(self.current_position.copy(), target_position, speed)
            )
            self.movement_thread.start()
            
            if wait:
                self.movement_thread.join()
                return True
            else:
                return True
        
        # Реальное перемещение робота
        data = {
            "x": x, "y": y, "z": z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "speed": speed,
            "wait": wait
        }
        
        response = self._send_command("MOVE_TO_POSITION", data)
        
        if response and response.get("status") == "ok":
            # Обновляем текущую позицию, если перемещение успешно
            if wait or not response.get("async", False):
                self.current_position = {
                    "x": x, "y": y, "z": z,
                    "roll": roll, "pitch": pitch, "yaw": yaw
                }
                self.current_joint_angles = joint_angles
            
            return True
        else:
            logger.error("Ошибка перемещения робота в указанную позицию")
            return False
    
    def move_joints(self, joint_angles: List[float], speed: float = None, wait: bool = True) -> bool:
        """
        Перемещение суставов робота в указанные углы.
        
        Args:
            joint_angles: Список целевых углов суставов
            speed: Скорость перемещения (% от максимальной)
            wait: Ожидать ли завершения перемещения
            
        Returns:
            bool: True, если перемещение выполнено успешно, иначе False
        """
        # Проверка на допустимость углов суставов
        if not validate_joint_angles(joint_angles, self.joint_limits):
            logger.error("Указанные углы суставов выходят за допустимые пределы")
            return False
        
        # Расчет позиции (прямая кинематика)
        cartesian = joint_to_cartesian(joint_angles)
        
        if cartesian is None:
            logger.error("Не удалось рассчитать позицию для указанных углов суставов")
            return False
        
        x, y, z, orientation = cartesian
        roll, pitch, yaw = orientation
        
        # Проверка на допустимость координат
        if not validate_position(x, y, z, self.workspace_limits):
            logger.error(f"Координаты ({x}, {y}, {z}) вне рабочей зоны")
            return False
        
        # Устанавливаем скорость перемещения
        if speed is None:
            speed = self.max_speed
        else:
            speed = min(speed, self.max_speed)  # Ограничиваем максимальной скоростью
        
        if self.emulation_mode:
            # Эмуляция перемещения
            if self.movement_thread and self.movement_thread.is_alive():
                # Если уже идет перемещение, останавливаем его
                self.stop_movement.set()
                self.movement_thread.join()
                self.stop_movement.clear()
            
            # Запускаем новое перемещение
            target_position = {
                "x": x, "y": y, "z": z,
                "roll": roll, "pitch": pitch, "yaw": yaw
            }
            
            self.movement_thread = threading.Thread(
                target=self._emulate_movement,
                args=(self.current_position.copy(), target_position, speed)
            )
            self.movement_thread.start()
            
            if wait:
                self.movement_thread.join()
                return True
            else:
                return True
        
        # Реальное перемещение робота
        data = {
            "angles": joint_angles,
            "speed": speed,
            "wait": wait
        }
        
        response = self._send_command("MOVE_JOINTS", data)
        
        if response and response.get("status") == "ok":
            # Обновляем текущие углы и позицию, если перемещение успешно
            if wait or not response.get("async", False):
                self.current_joint_angles = joint_angles
                self.current_position = {
                    "x": x, "y": y, "z": z,
                    "roll": roll, "pitch": pitch, "yaw": yaw
                }
            
            return True
        else:
            logger.error("Ошибка перемещения суставов робота")
            return False
    
    def move_gripper(self, position: float, speed: float = None, wait: bool = True) -> bool:
        """
        Управление захватом робота.
        
        Args:
            position: Позиция захвата (0.0 - открыт, 100.0 - закрыт)
            speed: Скорость перемещения (% от максимальной)
            wait: Ожидать ли завершения перемещения
            
        Returns:
            bool: True, если перемещение выполнено успешно, иначе False
        """
        # Проверка на допустимость позиции захвата
        position = max(0.0, min(100.0, position))  # Ограничиваем диапазоном [0, 100]
        
        # Устанавливаем скорость перемещения
        if speed is None:
            speed = self.max_speed
        else:
            speed = min(speed, self.max_speed)  # Ограничиваем максимальной скоростью
        
        if self.emulation_mode:
            # Эмуляция перемещения захвата
            logger.info(f"Эмуляция перемещения захвата в позицию {position}")
            
            # Вычисляем время перемещения
            movement_time = abs(position - self.gripper_position) / speed * 0.1  # Примерный расчет времени
            
            if wait:
                time.sleep(movement_time)
            
            self.gripper_position = position
            return True
        
        # Реальное управление захватом
        data = {
            "position": position,
            "speed": speed,
            "wait": wait
        }
        
        response = self._send_command("MOVE_GRIPPER", data)
        
        if response and response.get("status") == "ok":
            # Обновляем текущую позицию захвата, если перемещение успешно
            if wait or not response.get("async", False):
                self.gripper_position = position
            
            return True
        else:
            logger.error("Ошибка управления захватом робота")
            return False
    
    def stop(self) -> bool:
        """
        Экстренная остановка робота.
        
        Returns:
            bool: True, если остановка выполнена успешно, иначе False
        """
        if self.emulation_mode:
            # Останавливаем эмуляцию движения
            if self.movement_thread and self.movement_thread.is_alive():
                self.stop_movement.set()
                self.movement_thread.join()
                self.stop_movement.clear()
            
            logger.info("Эмуляция остановки робота")
            return True
        
        # Реальная остановка робота
        response = self._send_command("STOP")
        
        if response and response.get("status") == "ok":
            logger.info("Робот успешно остановлен")
            return True
        else:
            logger.error("Ошибка остановки робота")
            return False
    
    def reset(self) -> bool:
        """
        Сброс ошибок робота и возврат в исходное состояние.
        
        Returns:
            bool: True, если сброс выполнен успешно, иначе False
        """
        if self.emulation_mode:
            logger.info("Эмуляция сброса ошибок робота")
            return True
        
        # Реальный сброс ошибок робота
        response = self._send_command("RESET")
        
        if response and response.get("status") == "ok":
            logger.info("Робот успешно сброшен")
            return True
        else:
            logger.error("Ошибка сброса робота")
            return False
    
    def home(self) -> bool:
        """
        Перемещение робота в домашнюю позицию.
        
        Returns:
            bool: True, если перемещение выполнено успешно, иначе False
        """
        # Получаем домашнюю позицию из конфигурации
        home_position = self.config.get("HOME_POSITION", {
            "x": 0.0, "y": 0.0, "z": 200.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0
        })
        
        return self.move_to_position(
            home_position.get("x", 0.0),
            home_position.get("y", 0.0),
            home_position.get("z", 200.0),
            home_position.get("roll", 0.0),
            home_position.get("pitch", 0.0),
            home_position.get("yaw", 0.0),
            wait=True
        )
    
    def run_script(self, script_name: str, parameters: Dict[str, Any] = None) -> bool:
        """
        Запуск предопределенного скрипта на роботе.
        
        Args:
            script_name: Название скрипта для запуска
            parameters: Параметры для передачи скрипту
            
        Returns:
            bool: True, если скрипт выполнен успешно, иначе False
        """
        # Проверка наличия скрипта
        scripts = self.config.get("SCRIPTS", {})
        script = scripts.get(script_name)
        
        if not script:
            logger.error(f"Скрипт '{script_name}' не найден")
            return False
        
        logger.info(f"Запуск скрипта '{script_name}'")
        
        # Параметры по умолчанию
        default_params = script.get("parameters", {})
        
        # Объединяем параметры по умолчанию с переданными
        if parameters:
            # Создаем копию параметров по умолчанию
            merged_params = default_params.copy()
            # Обновляем переданными параметрами
            merged_params.update(parameters)
        else:
            merged_params = default_params
        
        # Получаем действия скрипта
        actions = script.get("actions", [])
        
        if not actions:
            logger.warning(f"Скрипт '{script_name}' не содержит действий")
            return True
        
        # Выполняем действия по порядку
        for action in actions:
            action_type = action.get("type")
            action_params = action.get("parameters", {})
            
            # Заменяем параметры из merged_params
            for key, value in action_params.items():
                if isinstance(value, str) and value.startswith("$"):
                    param_name = value[1:]  # Удаляем '$' из начала строки
                    if param_name in merged_params:
                        action_params[key] = merged_params.get(param_name)
            
            # Выполняем действие в зависимости от типа
            if action_type == "move_to_position":
                success = self.move_to_position(
                    action_params.get("x", 0.0),
                    action_params.get("y", 0.0),
                    action_params.get("z", 0.0),
                    action_params.get("roll", 0.0),
                    action_params.get("pitch", 0.0),
                    action_params.get("yaw", 0.0),
                    action_params.get("speed", None),
                    action_params.get("wait", True)
                )
            elif action_type == "move_joints":
                success = self.move_joints(
                    action_params.get("angles", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    action_params.get("speed", None),
                    action_params.get("wait", True)
                )
            elif action_type == "move_gripper":
                success = self.move_gripper(
                    action_params.get("position", 0.0),
                    action_params.get("speed", None),
                    action_params.get("wait", True)
                )
            elif action_type == "wait":
                time.sleep(action_params.get("time", 1.0))
                success = True
            elif action_type == "home":
                success = self.home()
            else:
                logger.warning(f"Неизвестный тип действия: {action_type}")
                success = False
            
            if not success:
                logger.error(f"Ошибка выполнения действия '{action_type}' в скрипте '{script_name}'")
                return False
        
        logger.info(f"Скрипт '{script_name}' успешно выполнен")
        return True
    
    def _emulate_movement(self, start_position: Dict[str, float], 
                          target_position: Dict[str, float], speed: float) -> None:
        """
        Эмуляция перемещения робота от начальной позиции к целевой.
        
        Args:
            start_position: Начальная позиция
            target_position: Целевая позиция
            speed: Скорость перемещения
        """
        # Создаем точки для интерполяции
        start_pos = [
            start_position.get("x", 0.0),
            start_position.get("y", 0.0),
            start_position.get("z", 0.0)
        ]
        
        target_pos = [
            target_position.get("x", 0.0),
            target_position.get("y", 0.0),
            target_position.get("z", 0.0)
        ]
        
        # Интерполируем движение
        points = interpolate_positions(start_pos, target_pos, 10)
        trajectory, total_time = calculate_trajectory(points, speed)
        
        # Имитация движения
        for point_data in trajectory:
            if self.stop_movement.is_set():
                logger.info("Эмуляция движения остановлена")
                return
            
            point = point_data["point"]
            segment_time = point_data["time"]
            
            # Обновляем текущую позицию
            self.current_position["x"] = point[0]
            self.current_position["y"] = point[1]
            self.current_position["z"] = point[2]
            
            # Линейная интерполяция ориентации
            t = 0.0 if total_time == 0.0 else segment_time / total_time
            self.current_position["roll"] = start_position["roll"] + t * (target_position["roll"] - start_position["roll"])
            self.current_position["pitch"] = start_position["pitch"] + t * (target_position["pitch"] - start_position["pitch"])
            self.current_position["yaw"] = start_position["yaw"] + t * (target_position["yaw"] - start_position["yaw"])
            
            # Обновляем также и углы суставов
            orientation = [
                self.current_position["roll"],
                self.current_position["pitch"],
                self.current_position["yaw"]
            ]
            self.current_joint_angles = cartesian_to_joint(
                self.current_position["x"],
                self.current_position["y"],
                self.current_position["z"],
                orientation
            ) or self.current_joint_angles
            
            # Имитация задержки движения
            if segment_time > 0:
                time.sleep(segment_time)
        
        # Устанавливаем конечную позицию
        self.current_position = target_position.copy()
        logger.debug(f"Эмуляция движения завершена: {self.current_position}")


# Пример использования
if __name__ == "__main__":
    # Настройка логирования
    logging.basicConfig(level=logging.INFO)
    
    # Создание конфигурации
    config = {
        "EMULATION_MODE": True,
        "ROBOT_IP": "192.168.0.100",
        "ROBOT_PORT": 5000,
        "MAX_SPEED": 50.0,
        "HOME_POSITION": {
            "x": 0.0, "y": 0.0, "z": 200.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0
        },
        "SCRIPTS": {
            "simple_demo": {
                "actions": [
                    {"type": "home"},
                    {"type": "move_to_position", "parameters": {"x": 100.0, "y": 0.0, "z": 200.0}},
                    {"type": "move_gripper", "parameters": {"position": 50.0}},
                    {"type": "wait", "parameters": {"time": 1.0}},
                    {"type": "move_gripper", "parameters": {"position": 0.0}},
                    {"type": "home"}
                ]
            }
        }
    }
    
    # Создание API робота
    robot = RobotAPI(config)
    
    # Подключение к роботу
    if robot.connect():
        try:
            # Перемещение в домашнюю позицию
            robot.home()
            
            # Запуск демо-скрипта
            robot.run_script("simple_demo")
            
        finally:
            # Отключение от робота
            robot.disconnect() 