#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Пример интеграции робота с компьютерным зрением.
Этот скрипт демонстрирует использование камеры для обнаружения объектов
и управление роботом для их захвата и сортировки.
"""

import os
import sys
import time
import logging
import cv2
import numpy as np
from datetime import datetime
import argparse

# Импортируем конфигурационный файл
import config

# Инициализация логирования
def setup_logger():
    """Настройка системы логирования."""
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL),
        format=log_format,
        handlers=[
            logging.FileHandler(config.LOG_FILE),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger("RobotVision")

logger = setup_logger()

class RobotController:
    """Класс для управления роботом."""
    
    def __init__(self, emulation=False):
        """
        Инициализация контроллера робота.
        
        Args:
            emulation (bool): Включить режим эмуляции.
        """
        self.emulation = emulation
        self.connected = False
        self.logger = logging.getLogger("RobotVision.RobotController")
        self.logger.info("Инициализация контроллера робота (эмуляция: %s)", emulation)
        
        if not emulation:
            try:
                # В реальной реализации здесь должен быть импорт RISDK
                # и инициализация соединения с роботом
                # from risdk import RobotAPI
                # self.robot = RobotAPI()
                self.logger.info("Подключение к роботу: %s:%s", 
                                config.ROBOT_IP, config.ROBOT_PORT)
                # self.robot.connect(config.ROBOT_IP, config.ROBOT_PORT)
                # self.robot.set_speed(config.MAX_SPEED)
                # self.robot.set_acceleration(config.MAX_ACCELERATION)
                self.connected = True
                self.logger.info("Подключение к роботу успешно установлено")
            except Exception as e:
                self.logger.error("Ошибка подключения к роботу: %s", str(e))
                raise
        else:
            self.logger.info("Работа в режиме эмуляции")
            self.connected = True
    
    def __del__(self):
        """Деструктор для освобождения ресурсов."""
        self.disconnect()
    
    def disconnect(self):
        """Отключение от робота."""
        if not self.emulation and self.connected:
            self.logger.info("Отключение от робота")
            # В реальной реализации здесь должен быть код отключения от робота
            # self.robot.disconnect()
            self.connected = False
    
    def move_to_position(self, position):
        """
        Перемещение робота в указанную позицию.
        
        Args:
            position (dict): Словарь с координатами {x, y, z, roll, pitch, yaw}.
        """
        if not self.connected:
            self.logger.error("Робот не подключен")
            return False
        
        self.logger.info("Перемещение в позицию: %s", position)
        
        if self.emulation:
            # В режиме эмуляции просто ждем
            time.sleep(1.0)
            self.logger.info("Эмуляция: робот переместился в позицию")
            return True
        else:
            try:
                # В реальной реализации здесь должен быть код перемещения робота
                # self.robot.move_to_position(
                #     position["x"], position["y"], position["z"],
                #     position["roll"], position["pitch"], position["yaw"]
                # )
                return True
            except Exception as e:
                self.logger.error("Ошибка при перемещении: %s", str(e))
                return False
    
    def move_to_home(self):
        """Перемещение робота в домашнюю позицию."""
        return self.move_to_position(config.HOME_POSITION)
    
    def move_to_standby(self):
        """Перемещение робота в позицию ожидания."""
        return self.move_to_position(config.STANDBY_POSITION)
    
    def control_gripper(self, open_gripper):
        """
        Управление захватом робота.
        
        Args:
            open_gripper (bool): True для открытия, False для закрытия.
        """
        if not self.connected:
            self.logger.error("Робот не подключен")
            return False
        
        position = config.GRIPPER_OPEN_POSITION if open_gripper else config.GRIPPER_CLOSED_POSITION
        self.logger.info("Управление захватом: %s (позиция: %s)", 
                        "открытие" if open_gripper else "закрытие", position)
        
        if self.emulation:
            # В режиме эмуляции просто ждем
            time.sleep(0.5)
            self.logger.info("Эмуляция: захват установлен в позицию %s", position)
            return True
        else:
            try:
                # В реальной реализации здесь должен быть код управления захватом
                # self.robot.set_gripper_position(position)
                return True
            except Exception as e:
                self.logger.error("Ошибка при управлении захватом: %s", str(e))
                return False
    
    def open_gripper(self):
        """Открытие захвата."""
        return self.control_gripper(True)
    
    def close_gripper(self):
        """Закрытие захвата."""
        return self.control_gripper(False)
    
    def pick_object(self, position):
        """
        Захват объекта в указанной позиции.
        
        Args:
            position (dict): Словарь с координатами объекта {x, y, z}.
        """
        # Создаем полную позицию из координат объекта
        target_position = {
            "x": position["x"],
            "y": position["y"],
            "z": position["z"] + config.SORTING_PARAMETERS["grasp_height"],
            "roll": 0,
            "pitch": 90,  # Захват сверху
            "yaw": 0
        }
        
        # Позиция подхода (для избежания столкновений)
        approach_position = target_position.copy()
        approach_position["z"] += config.SORTING_PARAMETERS["approach_offset"]
        
        self.logger.info("Захват объекта в позиции: %s", position)
        
        # Последовательность действий для захвата
        if not self.move_to_position(approach_position):
            return False
        
        if not self.open_gripper():
            return False
        
        if not self.move_to_position(target_position):
            return False
        
        if not self.close_gripper():
            return False
        
        # Небольшая пауза для стабилизации захвата
        time.sleep(config.SORTING_PARAMETERS["grasp_delay"])
        
        # Подъем с объектом
        if not self.move_to_position(approach_position):
            return False
        
        return True
    
    def place_object(self, position):
        """
        Размещение объекта в указанной позиции.
        
        Args:
            position (dict): Словарь с координатами для размещения {x, y, z}.
        """
        # Создаем полную позицию из координат размещения
        target_position = {
            "x": position["x"],
            "y": position["y"],
            "z": position["z"],
            "roll": 0,
            "pitch": 90,  # Размещение сверху
            "yaw": 0
        }
        
        # Позиция подхода (для избежания столкновений)
        approach_position = target_position.copy()
        approach_position["z"] += config.SORTING_PARAMETERS["approach_offset"]
        
        self.logger.info("Размещение объекта в позиции: %s", position)
        
        # Последовательность действий для размещения
        if not self.move_to_position(approach_position):
            return False
        
        if not self.move_to_position(target_position):
            return False
        
        if not self.open_gripper():
            return False
        
        # Небольшая пауза для стабилизации
        time.sleep(config.SORTING_PARAMETERS["grasp_delay"])
        
        # Отъезд от точки размещения
        if not self.move_to_position(approach_position):
            return False
        
        return True


class VisionSystem:
    """Класс для работы с компьютерным зрением."""
    
    def __init__(self):
        """Инициализация системы компьютерного зрения."""
        self.logger = logging.getLogger("RobotVision.VisionSystem")
        self.logger.info("Инициализация системы компьютерного зрения")
        
        # Инициализация камеры
        self.camera = None
        self.frame = None
        self.frame_time = None
        
        # Параметры для записи видео
        self.video_writer = None
        
        # Загрузка модели YOLO
        self.net = None
        self.layer_names = None
        self.output_layers = None
        self.classes = []
        
        # Загрузка классов объектов
        self.load_classes()
        
        # Загрузка модели нейронной сети
        self.load_model()
    
    def load_classes(self):
        """Загрузка классов объектов."""
        try:
            with open(config.YOLO_CONFIG["classes_path"], "r") as f:
                self.classes = [line.strip() for line in f.readlines()]
            self.logger.info("Загружено %d классов объектов", len(self.classes))
        except Exception as e:
            self.logger.error("Ошибка загрузки классов: %s", str(e))
            # Установка базовых классов при ошибке
            self.classes = ["person", "bicycle", "car", "motorbike", "aeroplane", 
                           "bus", "train", "truck", "boat", "bottle"]
            self.logger.warning("Использование базового набора классов")
    
    def load_model(self):
        """Загрузка модели нейронной сети для детекции объектов."""
        try:
            # Проверка наличия файлов модели
            if not os.path.exists(config.YOLO_CONFIG["config_path"]):
                self.logger.error("Файл конфигурации модели не найден: %s", 
                                 config.YOLO_CONFIG["config_path"])
                return False
            
            if not os.path.exists(config.YOLO_CONFIG["model_path"]):
                self.logger.error("Файл весов модели не найден: %s", 
                                 config.YOLO_CONFIG["model_path"])
                return False
            
            # Загрузка модели
            self.logger.info("Загрузка модели YOLO")
            self.net = cv2.dnn.readNet(
                config.YOLO_CONFIG["model_path"], 
                config.YOLO_CONFIG["config_path"]
            )
            
            # Получение выходных слоев
            layer_names = self.net.getLayerNames()
            output_layers_indices = self.net.getUnconnectedOutLayers()
            
            # В зависимости от версии OpenCV, getUnconnectedOutLayers может возвращать
            # разные форматы индексов
            if len(output_layers_indices.shape) > 1:
                output_layers_indices = output_layers_indices.flatten()
            
            # OpenCV возвращает индексы начиная с 1, но в Python индексация с 0
            self.output_layers = [layer_names[i - 1] for i in output_layers_indices]
            
            self.logger.info("Модель YOLO успешно загружена")
            return True
        except Exception as e:
            self.logger.error("Ошибка загрузки модели: %s", str(e))
            return False
    
    def start_camera(self):
        """Запуск камеры."""
        try:
            self.logger.info("Запуск камеры (индекс: %d)", config.CAMERA_INDEX)
            self.camera = cv2.VideoCapture(config.CAMERA_INDEX)
            
            # Установка параметров камеры
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            self.camera.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
            
            # Проверка успешного открытия камеры
            if not self.camera.isOpened():
                self.logger.error("Не удалось открыть камеру")
                return False
            
            # Считывание первого кадра для проверки
            ret, frame = self.camera.read()
            if not ret or frame is None:
                self.logger.error("Не удалось получить кадр с камеры")
                return False
            
            self.logger.info("Камера успешно запущена")
            
            # Инициализация записи видео, если включено
            if config.VISION_PARAMETERS["record_video"]:
                self.start_video_recording()
            
            return True
        except Exception as e:
            self.logger.error("Ошибка при запуске камеры: %s", str(e))
            return False
    
    def start_video_recording(self):
        """Начало записи видео."""
        try:
            # Получение размеров кадра
            ret, frame = self.camera.read()
            if not ret or frame is None:
                self.logger.error("Не удалось получить кадр для инициализации записи видео")
                return False
            
            height, width, _ = frame.shape
            
            # Создание записывающего объекта
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_path = f"{timestamp}_{config.VISION_PARAMETERS['video_output_path']}"
            
            self.video_writer = cv2.VideoWriter(
                video_path, fourcc, config.CAMERA_FPS, (width, height)
            )
            
            self.logger.info("Запись видео начата: %s", video_path)
            return True
        except Exception as e:
            self.logger.error("Ошибка при инициализации записи видео: %s", str(e))
            return False
    
    def stop_camera(self):
        """Остановка камеры и освобождение ресурсов."""
        if self.camera is not None:
            self.logger.info("Остановка камеры")
            
            # Остановка записи видео, если активна
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
                self.logger.info("Запись видео остановлена")
            
            # Освобождение ресурсов камеры
            self.camera.release()
            self.camera = None
            
            # Закрытие всех окон
            cv2.destroyAllWindows()
            
            self.logger.info("Камера остановлена")
    
    def capture_frame(self):
        """
        Захват кадра с камеры.
        
        Returns:
            bool: True, если кадр успешно захвачен.
        """
        if self.camera is None:
            self.logger.error("Камера не инициализирована")
            return False
        
        try:
            ret, frame = self.camera.read()
            if not ret or frame is None:
                self.logger.error("Не удалось получить кадр с камеры")
                return False
            
            self.frame = frame
            self.frame_time = time.time()
            
            # Запись кадра в видео, если активно
            if self.video_writer is not None:
                self.video_writer.write(frame)
            
            return True
        except Exception as e:
            self.logger.error("Ошибка при захвате кадра: %s", str(e))
            return False
    
    def detect_objects(self, frame=None):
        """
        Обнаружение объектов на кадре.
        
        Args:
            frame (numpy.ndarray, optional): Кадр для обработки.
                Если None, используется последний захваченный кадр.
                
        Returns:
            list: Список обнаруженных объектов в формате:
                [
                    {
                        "class_id": int,     # ID класса объекта
                        "class_name": str,   # Название класса объекта
                        "confidence": float, # Уверенность обнаружения (0-1)
                        "box": (x, y, w, h), # Ограничивающая рамка (x, y, ширина, высота)
                        "center": (x, y)     # Центр объекта
                    },
                    ...
                ]
        """
        # Если кадр не передан, используем последний захваченный
        if frame is None:
            if self.frame is None:
                self.logger.error("Кадр не доступен для обработки")
                return []
            frame = self.frame
        
        if self.net is None:
            self.logger.error("Модель YOLO не загружена")
            return []
        
        height, width, _ = frame.shape
        
        try:
            # Предобработка кадра для YOLO
            blob = cv2.dnn.blobFromImage(
                frame, 1/255.0, (416, 416), swapRB=True, crop=False
            )
            
            # Прямой проход через нейронную сеть
            self.net.setInput(blob)
            outputs = self.net.forward(self.output_layers)
            
            # Списки для хранения результатов
            boxes = []
            confidences = []
            class_ids = []
            
            # Обработка результатов
            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    
                    # Фильтрация по порогу уверенности
                    if confidence > config.YOLO_CONFIG["confidence_threshold"]:
                        # Координаты объекта масштабируются к размеру кадра
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Координаты левого верхнего угла
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        boxes.append((x, y, w, h))
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Применение non-maximum suppression (NMS)
            indices = cv2.dnn.NMSBoxes(
                boxes, confidences, 
                config.YOLO_CONFIG["confidence_threshold"],
                config.YOLO_CONFIG["nms_threshold"]
            )
            
            # Формирование результата
            result = []
            
            # OpenCV 4.5.4+ возвращает индексы в другом формате
            if len(indices) > 0:
                if isinstance(indices, tuple):
                    indices = indices[0]  # для старых версий OpenCV
                
                for i in indices.flatten():
                    box = boxes[i]
                    x, y, w, h = box
                    
                    # Получение класса объекта
                    class_id = class_ids[i]
                    class_name = self.classes[class_id] if class_id < len(self.classes) else "unknown"
                    
                    # Центр объекта
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    result.append({
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": confidences[i],
                        "box": box,
                        "center": (center_x, center_y)
                    })
            
            self.logger.info("Обнаружено %d объектов", len(result))
            return result
        except Exception as e:
            self.logger.error("Ошибка при обнаружении объектов: %s", str(e))
            return []
    
    def visualize_detections(self, frame, detections):
        """
        Визуализация обнаруженных объектов на кадре.
        
        Args:
            frame (numpy.ndarray): Кадр для визуализации.
            detections (list): Список обнаруженных объектов.
            
        Returns:
            numpy.ndarray: Кадр с визуализацией.
        """
        # Копирование кадра для визуализации
        vis_frame = frame.copy()
        
        # Цвета для разных классов
        colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
        
        # Отрисовка каждого обнаруженного объекта
        for detection in detections:
            x, y, w, h = detection["box"]
            class_id = detection["class_id"]
            class_name = detection["class_name"]
            confidence = detection["confidence"]
            
            # Выбор цвета
            color = colors[class_id % len(colors)]
            
            # Отрисовка ограничивающей рамки
            cv2.rectangle(vis_frame, (x, y), (x + w, y + h), color, 2)
            
            # Отрисовка метки класса с уверенностью
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(vis_frame, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Отрисовка центра объекта
            center_x, center_y = detection["center"]
            cv2.circle(vis_frame, (center_x, center_y), 5, color, -1)
        
        return vis_frame
    
    def convert_to_robot_coordinates(self, pixel_x, pixel_y):
        """
        Преобразование координат пикселей в координаты робота.
        
        Args:
            pixel_x (int): X-координата в пикселях.
            pixel_y (int): Y-координата в пикселях.
            
        Returns:
            tuple: (x, y, z) координаты в системе координат робота.
        """
        # Получение параметров калибровки
        camera_matrix = np.array(config.CAMERA_CALIBRATION["camera_matrix"])
        transform_matrix = np.array(config.CAMERA_CALIBRATION["transform_matrix"])
        
        # В реальной системе здесь должен быть сложный код преобразования
        # Упрощенная версия для примера
        
        # Преобразование координат пикселя в нормализованные координаты камеры
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        
        # Нормализованные координаты в системе камеры
        x_norm = (pixel_x - cx) / fx
        y_norm = (pixel_y - cy) / fy
        
        # Примерное расстояние до объекта (в реальной системе определяется иначе)
        z_cam = 500  # мм
        
        # Точка в системе координат камеры
        point_cam = np.array([x_norm * z_cam, y_norm * z_cam, z_cam, 1])
        
        # Преобразование в систему координат робота
        point_robot = transform_matrix.dot(point_cam)
        
        return point_robot[0], point_robot[1], point_robot[2]


class RobotVisionApp:
    """Основной класс приложения для интеграции робота с компьютерным зрением."""
    
    def __init__(self, emulation=False):
        """
        Инициализация приложения.
        
        Args:
            emulation (bool): Включить режим эмуляции робота.
        """
        self.logger = logging.getLogger("RobotVision.App")
        self.logger.info("Инициализация приложения Robot Vision")
        
        # Создание объектов для работы с роботом и зрением
        self.robot = RobotController(emulation)
        self.vision = VisionSystem()
        
        # Флаг для контроля выполнения
        self.running = False
    
    def initialize(self):
        """Инициализация всех подсистем."""
        self.logger.info("Инициализация подсистем")
        
        # Инициализация системы компьютерного зрения
        if not self.vision.start_camera():
            self.logger.error("Не удалось инициализировать камеру")
            return False
        
        # Перемещение робота в домашнюю позицию
        if not self.robot.move_to_home():
            self.logger.error("Не удалось переместить робота в домашнюю позицию")
            return False
        
        self.running = True
        return True
    
    def cleanup(self):
        """Очистка ресурсов и завершение работы."""
        self.logger.info("Завершение работы приложения")
        
        # Остановка камеры
        self.vision.stop_camera()
        
        # Перемещение робота в безопасную позицию и отключение
        if self.robot.connected:
            self.robot.move_to_standby()
            self.robot.disconnect()
        
        self.running = False
    
    def run_vision_demo(self, duration=60):
        """
        Запуск демонстрации компьютерного зрения.
        
        Args:
            duration (int): Продолжительность демонстрации в секундах.
        """
        self.logger.info("Запуск демонстрации компьютерного зрения (длительность: %d сек)", 
                         duration)
        
        if not self.initialize():
            self.logger.error("Не удалось инициализировать системы")
            return False
        
        start_time = time.time()
        last_detect_time = 0
        detect_interval = config.DEMO_SCENARIOS["vision"]["detect_interval"]
        
        try:
            while self.running and (time.time() - start_time < duration):
                # Захват кадра
                if not self.vision.capture_frame():
                    self.logger.error("Ошибка при захвате кадра")
                    continue
                
                current_time = time.time()
                
                # Обнаружение объектов с заданной периодичностью
                if current_time - last_detect_time >= detect_interval:
                    detections = self.vision.detect_objects()
                    last_detect_time = current_time
                    
                    # Визуализация обнаружений
                    vis_frame = self.vision.visualize_detections(self.vision.frame, detections)
                    
                    # Вывод информации о каждом объекте
                    for i, detection in enumerate(detections):
                        class_name = detection["class_name"]
                        confidence = detection["confidence"]
                        center_x, center_y = detection["center"]
                        
                        # Преобразование в координаты робота
                        robot_x, robot_y, robot_z = self.vision.convert_to_robot_coordinates(
                            center_x, center_y
                        )
                        
                        self.logger.info(
                            "Объект %d: %s (%.2f), центр: (%d, %d), координаты робота: (%.1f, %.1f, %.1f)",
                            i+1, class_name, confidence, center_x, center_y, robot_x, robot_y, robot_z
                        )
                    
                    # Отображение кадра
                    cv2.imshow("Robot Vision Demo", vis_frame)
                
                # Обработка нажатий клавиш
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # Esc
                    self.logger.info("Выход по нажатию клавиши Esc")
                    break
            
            self.logger.info("Демонстрация компьютерного зрения завершена")
            return True
        except KeyboardInterrupt:
            self.logger.info("Демонстрация прервана пользователем")
            return False
        except Exception as e:
            self.logger.error("Ошибка при выполнении демонстрации: %s", str(e))
            return False
        finally:
            self.cleanup()
    
    def run_sorting_demo(self):
        """
        Запуск демонстрации сортировки объектов.
        """
        self.logger.info("Запуск демонстрации сортировки объектов")
        
        if not self.initialize():
            self.logger.error("Не удалось инициализировать системы")
            return False
        
        num_objects = config.DEMO_SCENARIOS["sorting"]["num_objects"]
        max_attempts = config.DEMO_SCENARIOS["sorting"]["max_attempts"]
        objects_sorted = 0
        
        try:
            # Открытие захвата перед началом
            self.robot.open_gripper()
            
            attempt = 0
            while self.running and objects_sorted < num_objects and attempt < max_attempts:
                attempt += 1
                self.logger.info("Попытка сортировки %d/%d", attempt, max_attempts)
                
                # Захват кадра
                if not self.vision.capture_frame():
                    self.logger.error("Ошибка при захвате кадра")
                    continue
                
                # Обнаружение объектов
                detections = self.vision.detect_objects()
                
                # Визуализация обнаружений
                vis_frame = self.vision.visualize_detections(self.vision.frame, detections)
                cv2.imshow("Robot Sorting Demo", vis_frame)
                cv2.waitKey(1)
                
                if not detections:
                    self.logger.info("Объекты не обнаружены")
                    time.sleep(1)
                    continue
                
                # Выбор объекта для сортировки (первый в списке)
                detection = detections[0]
                class_name = detection["class_name"]
                center_x, center_y = detection["center"]
                
                # Преобразование в координаты робота
                robot_x, robot_y, robot_z = self.vision.convert_to_robot_coordinates(
                    center_x, center_y
                )
                
                # Создание позиции для захвата
                object_position = {
                    "x": robot_x,
                    "y": robot_y,
                    "z": robot_z
                }
                
                self.logger.info(
                    "Сортировка объекта: %s, позиция: (%.1f, %.1f, %.1f)",
                    class_name, robot_x, robot_y, robot_z
                )
                
                # Захват объекта
                if not self.robot.pick_object(object_position):
                    self.logger.error("Не удалось захватить объект")
                    continue
                
                # Определение позиции для размещения в зависимости от класса
                if class_name in config.SORTING_PARAMETERS["class_positions"]:
                    place_position = config.SORTING_PARAMETERS["class_positions"][class_name]
                else:
                    place_position = config.SORTING_PARAMETERS["class_positions"]["default"]
                
                # Размещение объекта
                if not self.robot.place_object(place_position):
                    self.logger.error("Не удалось разместить объект")
                    continue
                
                objects_sorted += 1
                self.logger.info(
                    "Объект успешно отсортирован (%d/%d)",
                    objects_sorted, num_objects
                )
                
                # Задержка перед следующей попыткой
                time.sleep(1)
            
            self.logger.info(
                "Демонстрация сортировки завершена. Отсортировано объектов: %d/%d",
                objects_sorted, num_objects
            )
            
            return objects_sorted > 0
        except KeyboardInterrupt:
            self.logger.info("Демонстрация прервана пользователем")
            return False
        except Exception as e:
            self.logger.error("Ошибка при выполнении демонстрации: %s", str(e))
            return False
        finally:
            self.cleanup()


def main():
    """Основная функция программы."""
    parser = argparse.ArgumentParser(description="Пример интеграции робота с компьютерным зрением")
    parser.add_argument("--emulation", action="store_true", help="Включить режим эмуляции робота")
    parser.add_argument("--demo", choices=["vision", "sorting"], default="vision",
                       help="Тип демонстрации (vision - только зрение, sorting - сортировка объектов)")
    parser.add_argument("--duration", type=int, default=60,
                       help="Продолжительность демонстрации в секундах (для режима vision)")
    
    args = parser.parse_args()
    
    # Создание приложения
    app = RobotVisionApp(emulation=args.emulation or config.EMULATION_MODE)
    
    # Запуск выбранной демонстрации
    if args.demo == "vision":
        app.run_vision_demo(args.duration)
    else:
        app.run_sorting_demo()


if __name__ == "__main__":
    main() 