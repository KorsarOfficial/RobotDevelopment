#!/bin/bash

# Создание пакета для графического интерфейса
cd ~/competitions_ws/src
ros2 pkg create --build-type ament_python robot_gui --dependencies rclpy std_msgs geometry_msgs sensor_msgs visualization_msgs

# Создание структуры пакета
mkdir -p ~/competitions_ws/src/robot_gui/robot_gui/ui
mkdir -p ~/competitions_ws/src/robot_gui/robot_gui/api
mkdir -p ~/competitions_ws/src/robot_gui/robot_gui/visualization
mkdir -p ~/competitions_ws/src/robot_gui/robot_gui/utils

# Создание основных файлов
touch ~/competitions_ws/src/robot_gui/robot_gui/__init__.py
touch ~/competitions_ws/src/robot_gui/robot_gui/ui/__init__.py
touch ~/competitions_ws/src/robot_gui/robot_gui/api/__init__.py
touch ~/competitions_ws/src/robot_gui/robot_gui/visualization/__init__.py
touch ~/competitions_ws/src/robot_gui/robot_gui/utils/__init__.py

# Создание файла main_window.py
cat > ~/competitions_ws/src/robot_gui/robot_gui/ui/main_window.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Главное окно графического интерфейса для управления роботизированной ячейкой.
"""

import sys
import os
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QGroupBox, 
                             QLineEdit, QGridLayout, QTabWidget, QTextEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QFont, QIcon

class RobotGUI(QMainWindow):
    """Главное окно интерфейса управления роботом."""
    
    def __init__(self, node):
        """Инициализация окна."""
        super().__init__()
        
        # Сохраняем ссылку на ROS2 ноду
        self.node = node
        
        # Настройка окна
        self.setWindowTitle("Управление роботизированной ячейкой")
        self.setGeometry(100, 100, 1200, 800)
        
        # Центральный виджет
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Основной макет
        main_layout = QVBoxLayout(central_widget)
        
        # Создание компонентов интерфейса
        self._create_status_bar()
        self._create_control_panel(main_layout)
        self._create_visualization_area(main_layout)
        self._create_log_area(main_layout)
        
        # Таймер для обновления UI
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # 10 Гц
        
        # Отображение окна
        self.show()
    
    def _create_status_bar(self):
        """Создание строки состояния."""
        self.statusBar().showMessage("Готов к работе")
    
    def _create_control_panel(self, parent_layout):
        """Создание панели управления."""
        control_group = QGroupBox("Панель управления")
        control_layout = QGridLayout()
        
        # Кнопки управления системой
        self.btn_connect = QPushButton("Подключиться")
        self.btn_disconnect = QPushButton("Отключиться")
        self.btn_emergency = QPushButton("АВАРИЙНАЯ ОСТАНОВКА")
        self.btn_emergency.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        
        # Кнопки управления манипулятором
        self.btn_home = QPushButton("Домашняя позиция")
        self.btn_move = QPushButton("Переместить")
        self.btn_grip = QPushButton("Захват")
        self.btn_release = QPushButton("Разжим")
        
        # Поля для ввода координат
        coord_group = QGroupBox("Координаты")
        coord_layout = QGridLayout()
        
        coord_layout.addWidget(QLabel("X:"), 0, 0)
        self.edit_x = QLineEdit("0.0")
        coord_layout.addWidget(self.edit_x, 0, 1)
        
        coord_layout.addWidget(QLabel("Y:"), 1, 0)
        self.edit_y = QLineEdit("0.0")
        coord_layout.addWidget(self.edit_y, 1, 1)
        
        coord_layout.addWidget(QLabel("Z:"), 2, 0)
        self.edit_z = QLineEdit("0.0")
        coord_layout.addWidget(self.edit_z, 2, 1)
        
        coord_group.setLayout(coord_layout)
        
        # Размещение элементов
        control_layout.addWidget(self.btn_connect, 0, 0)
        control_layout.addWidget(self.btn_disconnect, 0, 1)
        control_layout.addWidget(self.btn_emergency, 0, 2, 1, 2)
        
        control_layout.addWidget(self.btn_home, 1, 0)
        control_layout.addWidget(self.btn_move, 1, 1)
        control_layout.addWidget(self.btn_grip, 1, 2)
        control_layout.addWidget(self.btn_release, 1, 3)
        
        control_layout.addWidget(coord_group, 2, 0, 2, 4)
        
        control_group.setLayout(control_layout)
        parent_layout.addWidget(control_group)
    
    def _create_visualization_area(self, parent_layout):
        """Создание области визуализации."""
        visual_tabs = QTabWidget()
        
        # Вкладка для камеры 1
        camera1_widget = QWidget()
        camera1_layout = QVBoxLayout(camera1_widget)
        camera1_layout.addWidget(QLabel("Камера 1 - Изображение будет здесь"))
        visual_tabs.addTab(camera1_widget, "Камера 1")
        
        # Вкладка для камеры 2
        camera2_widget = QWidget()
        camera2_layout = QVBoxLayout(camera2_widget)
        camera2_layout.addWidget(QLabel("Камера 2 - Изображение будет здесь"))
        visual_tabs.addTab(camera2_widget, "Камера 2")
        
        # Вкладка для обработанного изображения
        processed_widget = QWidget()
        processed_layout = QVBoxLayout(processed_widget)
        processed_layout.addWidget(QLabel("Обработанное изображение будет здесь"))
        visual_tabs.addTab(processed_widget, "Обработка")
        
        parent_layout.addWidget(visual_tabs)
    
    def _create_log_area(self, parent_layout):
        """Создание области для логов."""
        log_group = QGroupBox("Логи системы")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        parent_layout.addWidget(log_group)
    
    def update_ui(self):
        """Обновление интерфейса."""
        # Здесь будет код обновления UI из данных ROS2
        pass
    
    def add_log(self, message):
        """Добавление сообщения в лог."""
        self.log_text.append(message)
    
    def closeEvent(self, event):
        """Обработка закрытия окна."""
        self.update_timer.stop()
        event.accept()


class GUINode(Node):
    """ROS2 нода для графического интерфейса."""
    
    def __init__(self):
        """Инициализация ноды."""
        super().__init__('robot_gui_node')
        self.get_logger().info('Графический интерфейс запущен')


def main(args=None):
    """Точка входа в программу."""
    rclpy.init(args=args)
    gui_node = GUINode()
    
    app = QApplication([])
    gui = RobotGUI(gui_node)
    
    # Запуск обработки ROS2 сообщений в отдельном потоке
    # (в реальном приложении нужно будет реализовать многопоточность)
    
    try:
        app.exec_()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOL

# Создание файла robot_api_client.py
cat > ~/competitions_ws/src/robot_gui/robot_gui/api/robot_api_client.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Клиент для взаимодействия с API робота.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

class RobotAPIClient:
    """Клиент для взаимодействия с API робота."""
    
    def __init__(self, node):
        """Инициализация клиента.
        
        Args:
            node: Нода ROS2 для коммуникации
        """
        self.node = node
        
        # Создание издателей (publishers)
        self.cmd_pose_pub = self.node.create_publisher(
            Pose, '/robot/cmd_pose', 10)
        
        self.gripper_cmd_pub = self.node.create_publisher(
            Bool, '/robot/gripper_cmd', 10)
        
        # Создание подписчиков (subscribers)
        self.joint_state_sub = self.node.create_subscription(
            JointState, 
            '/robot/joint_states', 
            self._joint_state_callback, 
            10)
        
        # Текущее состояние суставов робота
        self.current_joint_state = None
    
    def _joint_state_callback(self, msg):
        """Обработчик сообщений о состоянии суставов робота."""
        self.current_joint_state = msg
    
    def move_to_pose(self, x, y, z, orientation=None):
        """Переместить робота в указанную позицию.
        
        Args:
            x, y, z: Координаты
            orientation: Ориентация (кватернион)
        """
        # Создание сообщения с позицией
        pose_msg = Pose()
        pose_msg.position = Point(x=float(x), y=float(y), z=float(z))
        
        if orientation:
            pose_msg.orientation = orientation
        else:
            # Ориентация по умолчанию - без поворота
            pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Отправка команды
        self.cmd_pose_pub.publish(pose_msg)
    
    def set_gripper(self, close):
        """Управление грипером.
        
        Args:
            close: True - закрыть, False - открыть
        """
        # Отправка команды грипперу
        self.gripper_cmd_pub.publish(Bool(data=close))
    
    def get_current_pose(self):
        """Получить текущую позицию робота.
        
        Returns:
            Текущая позиция (x, y, z) или None, если данные недоступны
        """
        # В реальном проекте здесь будет прямая кинематика
        # или получение данных от соответствующего топика
        if self.current_joint_state:
            # Placeholder для демонстрации
            return (0.0, 0.0, 0.0)
        return None
EOL

# Создание файла logger.py
cat > ~/competitions_ws/src/robot_gui/robot_gui/utils/logger.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Система логирования действий и событий.
"""

import os
import time
import logging
from datetime import datetime

class RobotLogger:
    """Система логирования роботизированной ячейки."""
    
    def __init__(self, log_dir=None, log_level=logging.INFO):
        """Инициализация логгера.
        
        Args:
            log_dir: Директория для хранения логов
            log_level: Уровень логирования
        """
        # Настройка директории логов
        if log_dir is None:
            self.log_dir = os.path.join(os.path.expanduser('~'), 'robot_logs')
        else:
            self.log_dir = log_dir
        
        # Создание директории, если она не существует
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Создание файла лога с временной меткой
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(self.log_dir, f'robot_log_{timestamp}.log')
        
        # Настройка логгера
        self.logger = logging.getLogger('robot_logger')
        self.logger.setLevel(log_level)
        
        # Обработчик для файла
        file_handler = logging.FileHandler(log_file)
        file_formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s')
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)
        
        # Обработчик для консоли
        console_handler = logging.StreamHandler()
        console_formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s')
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)
        
        # Список последних логов для отображения в GUI
        self.recent_logs = []
        self.max_recent_logs = 100
    
    def info(self, message):
        """Логирование информационного сообщения."""
        self.logger.info(message)
        self._add_to_recent(message, 'INFO')
    
    def warning(self, message):
        """Логирование предупреждения."""
        self.logger.warning(message)
        self._add_to_recent(message, 'WARNING')
    
    def error(self, message):
        """Логирование ошибки."""
        self.logger.error(message)
        self._add_to_recent(message, 'ERROR')
    
    def debug(self, message):
        """Логирование отладочной информации."""
        self.logger.debug(message)
        self._add_to_recent(message, 'DEBUG')
    
    def critical(self, message):
        """Логирование критической ошибки."""
        self.logger.critical(message)
        self._add_to_recent(message, 'CRITICAL')
    
    def _add_to_recent(self, message, level):
        """Добавление сообщения в список последних логов."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        log_entry = f'{timestamp} [{level}] {message}'
        
        self.recent_logs.append(log_entry)
        
        # Ограничение количества хранимых логов
        if len(self.recent_logs) > self.max_recent_logs:
            self.recent_logs.pop(0)
    
    def get_recent_logs(self):
        """Получить список последних логов."""
        return self.recent_logs
EOL

# Создание файла visualization.py
cat > ~/competitions_ws/src/robot_gui/robot_gui/visualization/visualization.py << 'EOL'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Компоненты для визуализации состояния и данных.
"""

import cv2
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QObject

class CameraViewer(QWidget):
    """Виджет для отображения видеопотока с камеры."""
    
    def __init__(self, parent=None):
        """Инициализация виджета."""
        super().__init__(parent)
        
        # Настройка лэйаута
        self.layout = QVBoxLayout(self)
        
        # Метка для отображения изображения
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.image_label)
        
        # Текущее изображение
        self.current_image = None
    
    def update_image(self, image):
        """Обновление изображения.
        
        Args:
            image: Изображение в формате OpenCV (numpy array)
        """
        self.current_image = image
        
        # Преобразование изображения OpenCV в QImage
        if image is not None:
            # Конвертация из BGR в RGB
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Создание QImage
            height, width, channels = rgb_image.shape
            bytes_per_line = channels * width
            q_image = QImage(rgb_image.data, width, height, 
                             bytes_per_line, QImage.Format_RGB888)
            
            # Отображение изображения
            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(pixmap)
    
    def clear(self):
        """Очистка изображения."""
        self.image_label.clear()
        self.current_image = None


class ImageProcessor(QObject):
    """Класс для обработки изображений."""
    
    # Сигнал для отправки обработанного изображения
    processed_image = pyqtSignal(np.ndarray)
    
    def __init__(self):
        """Инициализация процессора."""
        super().__init__()
    
    @pyqtSlot(np.ndarray)
    def process_image(self, image):
        """Обработка изображения.
        
        Args:
            image: Исходное изображение
        """
        if image is None:
            return
        
        # Пример простой обработки - перевод в оттенки серого и обратно в RGB
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        # Отправка обработанного изображения
        self.processed_image.emit(rgb)
EOL

# Обновление setup.py для добавления точки входа
cat > ~/competitions_ws/src/robot_gui/setup.py << 'EOL'
from setuptools import setup

package_name = 'robot_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 
              f'{package_name}.ui',
              f'{package_name}.api',
              f'{package_name}.visualization',
              f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Graphical user interface for robot control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_gui = robot_gui.ui.main_window:main',
        ],
    },
)
EOL

# Обновление package.xml для добавления зависимостей
cat > ~/competitions_ws/src/robot_gui/package.xml << 'EOL'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_gui</name>
  <version>0.0.1</version>
  <description>Graphical user interface for robot control</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-pyqt5</exec_depend>
  <exec_depend>python3-numpy</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOL

echo "Пакет robot_gui создан успешно!" 