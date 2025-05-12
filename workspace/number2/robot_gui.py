#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Графический интерфейс для ручного управления роботом-манипулятором.
Соответствует требованиям Модуля A конкурсного задания.
"""

import sys
import os
import time
import logging
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QLabel, QGroupBox, 
                           QLineEdit, QGridLayout, QTabWidget, QTextEdit,
                           QSlider, QComboBox, QCheckBox, QMessageBox,
                           QSplitter, QFileDialog, QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QIcon, QColor, QPalette

# Импортируем наши модули
from robot_client import RobotClient
from logger import Logger
from visualization import RobotVisualization
import config

class RobotGUI(QMainWindow):
    """Главное окно графического интерфейса."""
    
    def __init__(self):
        """Инициализация окна."""
        super().__init__()
        
        # Инициализация объектов
        self.robot = RobotClient()
        self.logger = Logger()
        self.visualization = RobotVisualization()
        
        # Настройка окна
        self.setWindowTitle("Управление роботом-манипулятором")
        self.setGeometry(100, 100, 1280, 800)
        self.setMinimumSize(800, 600)
        
        # Центральный виджет
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Основной макет
        main_layout = QVBoxLayout(central_widget)
        
        # Создание компонентов интерфейса
        self._create_menu_bar()
        self._create_status_bar()
        self._create_main_splitter(main_layout)
        
        # Таймер для обновления UI
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(100)  # 10 Гц
        
        # Начальная настройка интерфейса
        self._setup_initial_state()
        
        # Отображение окна
        self.show()
        
        # Лог о запуске
        self.logger.info("Графический интерфейс запущен")
        self.update_log_display()
    
    def _create_menu_bar(self):
        """Создание главного меню."""
        menu_bar = self.menuBar()
        
        # Меню "Файл"
        file_menu = menu_bar.addMenu("Файл")
        
        # Подключиться к роботу
        connect_action = file_menu.addAction("Подключиться к роботу")
        connect_action.triggered.connect(self.connect_to_robot)
        
        # Отключиться от робота
        disconnect_action = file_menu.addAction("Отключиться от робота")
        disconnect_action.triggered.connect(self.disconnect_from_robot)
        
        file_menu.addSeparator()
        
        # Сохранить логи
        save_logs_action = file_menu.addAction("Сохранить логи")
        save_logs_action.triggered.connect(self.save_logs)
        
        file_menu.addSeparator()
        
        # Выход
        exit_action = file_menu.addAction("Выход")
        exit_action.triggered.connect(self.close)
        
        # Меню "Вид"
        view_menu = menu_bar.addMenu("Вид")
        
        # Показать/скрыть панели
        toggle_control_panel_action = view_menu.addAction("Панель управления")
        toggle_control_panel_action.setCheckable(True)
        toggle_control_panel_action.setChecked(True)
        
        toggle_visual_panel_action = view_menu.addAction("Панель визуализации")
        toggle_visual_panel_action.setCheckable(True)
        toggle_visual_panel_action.setChecked(True)
        
        toggle_log_panel_action = view_menu.addAction("Панель логов")
        toggle_log_panel_action.setCheckable(True)
        toggle_log_panel_action.setChecked(True)
        
        # Меню "Справка"
        help_menu = menu_bar.addMenu("Справка")
        
        # О программе
        about_action = help_menu.addAction("О программе")
        about_action.triggered.connect(self.show_about_dialog)
    
    def _create_status_bar(self):
        """Создание строки состояния."""
        self.status_label = QLabel("Готов к работе")
        self.statusBar().addPermanentWidget(self.status_label, 1)
        
        self.connection_label = QLabel("Не подключен")
        self.connection_label.setStyleSheet("color: red;")
        self.statusBar().addPermanentWidget(self.connection_label)
    
    def _create_main_splitter(self, parent_layout):
        """Создание главного сплиттера."""
        # Создаем главный сплиттер (горизонтальный)
        self.main_splitter = QSplitter(Qt.Horizontal)
        
        # Левая часть - контроль
        self.control_panel = QWidget()
        control_layout = QVBoxLayout(self.control_panel)
        self._create_connection_panel(control_layout)
        self._create_manual_control_panel(control_layout)
        self._create_automation_panel(control_layout)
        
        # Растягиваем последний элемент для заполнения пространства
        control_layout.addStretch(1)
        
        # Правая часть (вертикальный сплиттер)
        self.right_splitter = QSplitter(Qt.Vertical)
        
        # Верхняя правая часть - визуализация
        visual_panel = QWidget()
        visual_layout = QVBoxLayout(visual_panel)
        self._create_visualization_panel(visual_layout)
        
        # Нижняя правая часть - логи
        log_panel = QWidget()
        log_layout = QVBoxLayout(log_panel)
        self._create_log_panel(log_layout)
        
        # Добавляем верхнюю и нижнюю части в правый сплиттер
        self.right_splitter.addWidget(visual_panel)
        self.right_splitter.addWidget(log_panel)
        
        # Устанавливаем начальный размер для правого сплиттера
        self.right_splitter.setSizes([600, 200])
        
        # Добавляем левую и правую части в главный сплиттер
        self.main_splitter.addWidget(self.control_panel)
        self.main_splitter.addWidget(self.right_splitter)
        
        # Устанавливаем начальный размер для главного сплиттера
        self.main_splitter.setSizes([300, 700])
        
        # Добавляем главный сплиттер в родительский макет
        parent_layout.addWidget(self.main_splitter)
    
    def _create_connection_panel(self, parent_layout):
        """Создание панели подключения."""
        connection_group = QGroupBox("Подключение")
        connection_layout = QGridLayout()
        
        # Кнопка подключения
        self.btn_connect = QPushButton("Подключиться")
        self.btn_connect.clicked.connect(self.connect_to_robot)
        connection_layout.addWidget(self.btn_connect, 0, 0)
        
        # Кнопка отключения
        self.btn_disconnect = QPushButton("Отключиться")
        self.btn_disconnect.clicked.connect(self.disconnect_from_robot)
        self.btn_disconnect.setEnabled(False)
        connection_layout.addWidget(self.btn_disconnect, 0, 1)
        
        # Кнопка аварийной остановки
        self.btn_emergency = QPushButton("АВАРИЙНАЯ ОСТАНОВКА")
        self.btn_emergency.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_emergency.clicked.connect(self.emergency_stop)
        connection_layout.addWidget(self.btn_emergency, 1, 0, 1, 2)
        
        connection_group.setLayout(connection_layout)
        parent_layout.addWidget(connection_group)
    
    def _create_manual_control_panel(self, parent_layout):
        """Создание панели ручного управления."""
        manual_group = QGroupBox("Ручное управление")
        manual_layout = QVBoxLayout()
        
        # Переключатель режима координат
        coord_mode_layout = QHBoxLayout()
        coord_mode_layout.addWidget(QLabel("Режим координат:"))
        
        self.coord_mode_combo = QComboBox()
        self.coord_mode_combo.addItems(["Декартовы координаты", "Углы сервоприводов"])
        self.coord_mode_combo.currentIndexChanged.connect(self.on_coord_mode_changed)
        coord_mode_layout.addWidget(self.coord_mode_combo)
        
        manual_layout.addLayout(coord_mode_layout)
        
        # Стек для разных режимов ввода координат
        self.coord_stack = QTabWidget()
        
        # Вкладка для декартовых координат
        xyz_widget = QWidget()
        xyz_layout = QGridLayout(xyz_widget)
        
        xyz_layout.addWidget(QLabel("X:"), 0, 0)
        self.edit_x = QLineEdit("0.0")
        xyz_layout.addWidget(self.edit_x, 0, 1)
        
        xyz_layout.addWidget(QLabel("Y:"), 1, 0)
        self.edit_y = QLineEdit("0.0")
        xyz_layout.addWidget(self.edit_y, 1, 1)
        
        xyz_layout.addWidget(QLabel("Z:"), 2, 0)
        self.edit_z = QLineEdit("0.0")
        xyz_layout.addWidget(self.edit_z, 2, 1)
        
        self.coord_stack.addTab(xyz_widget, "XYZ")
        
        # Вкладка для углов сервоприводов
        angles_widget = QWidget()
        angles_layout = QGridLayout(angles_widget)
        
        self.angle_inputs = []
        for i in range(6):  # 6 сервоприводов
            angles_layout.addWidget(QLabel(f"Сервопривод {i+1}:"), i, 0)
            angle_edit = QLineEdit("0.0")
            self.angle_inputs.append(angle_edit)
            angles_layout.addWidget(angle_edit, i, 1)
        
        self.coord_stack.addTab(angles_widget, "Углы")
        
        manual_layout.addWidget(self.coord_stack)
        
        # Кнопки управления
        control_layout = QGridLayout()
        
        # Кнопка перемещения
        self.btn_move = QPushButton("Переместить")
        self.btn_move.clicked.connect(self.move_robot)
        control_layout.addWidget(self.btn_move, 0, 0, 1, 2)
        
        # Кнопка домашней позиции
        self.btn_home = QPushButton("Домашняя позиция")
        self.btn_home.clicked.connect(self.home_position)
        control_layout.addWidget(self.btn_home, 1, 0, 1, 2)
        
        # Кнопки управления гриппером
        self.btn_grip = QPushButton("Захват")
        self.btn_grip.clicked.connect(lambda: self.control_gripper(True))
        control_layout.addWidget(self.btn_grip, 2, 0)
        
        self.btn_release = QPushButton("Разжим")
        self.btn_release.clicked.connect(lambda: self.control_gripper(False))
        control_layout.addWidget(self.btn_release, 2, 1)
        
        manual_layout.addLayout(control_layout)
        
        manual_group.setLayout(manual_layout)
        parent_layout.addWidget(manual_group)
    
    def _create_automation_panel(self, parent_layout):
        """Создание панели автоматического управления."""
        automation_group = QGroupBox("Автоматический режим")
        automation_layout = QVBoxLayout()
        
        # Список предопределенных положений
        automation_layout.addWidget(QLabel("Предопределенные позиции:"))
        
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(["Позиция 1", "Позиция 2", "Позиция 3", "Позиция 4", "Позиция 5"])
        automation_layout.addWidget(self.preset_combo)
        
        # Кнопка выполнения
        self.btn_goto_preset = QPushButton("Перейти к выбранной позиции")
        self.btn_goto_preset.clicked.connect(self.goto_preset)
        automation_layout.addWidget(self.btn_goto_preset)
        
        # Чекбокс для запуска циклического режима
        self.chk_cycle = QCheckBox("Циклический режим")
        automation_layout.addWidget(self.chk_cycle)
        
        # Кнопка запуска автоматического режима
        self.btn_start_auto = QPushButton("Запустить автоматический режим")
        self.btn_start_auto.clicked.connect(self.start_auto_mode)
        automation_layout.addWidget(self.btn_start_auto)
        
        # Кнопка остановки
        self.btn_stop_auto = QPushButton("Остановить")
        self.btn_stop_auto.clicked.connect(self.stop_auto_mode)
        self.btn_stop_auto.setEnabled(False)
        automation_layout.addWidget(self.btn_stop_auto)
        
        automation_group.setLayout(automation_layout)
        parent_layout.addWidget(automation_group)
    
    def _create_visualization_panel(self, parent_layout):
        """Создание панели визуализации."""
        visual_tabs = QTabWidget()
        
        # Вкладка для основной визуализации робота
        robot_visual_widget = QWidget()
        robot_visual_layout = QVBoxLayout(robot_visual_widget)
        
        # Здесь будет виджет для отображения 3D модели робота
        self.robot_visual_placeholder = QLabel("Визуализация робота будет здесь")
        self.robot_visual_placeholder.setAlignment(Qt.AlignCenter)
        self.robot_visual_placeholder.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
        self.robot_visual_placeholder.setMinimumSize(400, 300)
        robot_visual_layout.addWidget(self.robot_visual_placeholder)
        
        visual_tabs.addTab(robot_visual_widget, "Робот")
        
        # Вкладка для камеры 1
        camera1_widget = QWidget()
        camera1_layout = QVBoxLayout(camera1_widget)
        
        self.camera1_placeholder = QLabel("Изображение с камеры 1 будет здесь")
        self.camera1_placeholder.setAlignment(Qt.AlignCenter)
        self.camera1_placeholder.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
        self.camera1_placeholder.setMinimumSize(400, 300)
        camera1_layout.addWidget(self.camera1_placeholder)
        
        visual_tabs.addTab(camera1_widget, "Камера 1")
        
        # Вкладка для камеры 2
        camera2_widget = QWidget()
        camera2_layout = QVBoxLayout(camera2_widget)
        
        self.camera2_placeholder = QLabel("Изображение с камеры 2 будет здесь")
        self.camera2_placeholder.setAlignment(Qt.AlignCenter)
        self.camera2_placeholder.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
        self.camera2_placeholder.setMinimumSize(400, 300)
        camera2_layout.addWidget(self.camera2_placeholder)
        
        visual_tabs.addTab(camera2_widget, "Камера 2")
        
        parent_layout.addWidget(visual_tabs)
    
    def _create_log_panel(self, parent_layout):
        """Создание панели для логов."""
        log_group = QGroupBox("Логи системы")
        log_layout = QVBoxLayout()
        
        # Текстовое поле для логов
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier New", 9))
        log_layout.addWidget(self.log_text)
        
        # Кнопки для работы с логами
        log_button_layout = QHBoxLayout()
        
        self.btn_clear_log = QPushButton("Очистить")
        self.btn_clear_log.clicked.connect(self.clear_logs)
        log_button_layout.addWidget(self.btn_clear_log)
        
        self.btn_save_log = QPushButton("Сохранить")
        self.btn_save_log.clicked.connect(self.save_logs)
        log_button_layout.addWidget(self.btn_save_log)
        
        log_layout.addLayout(log_button_layout)
        
        log_group.setLayout(log_layout)
        parent_layout.addWidget(log_group)
    
    def _setup_initial_state(self):
        """Начальная настройка состояния интерфейса."""
        # Отключаем кнопки управления, пока нет подключения
        self.btn_move.setEnabled(False)
        self.btn_home.setEnabled(False)
        self.btn_grip.setEnabled(False)
        self.btn_release.setEnabled(False)
        self.btn_goto_preset.setEnabled(False)
        self.btn_start_auto.setEnabled(False)
        
        # Начальный режим - декартовы координаты
        self.coord_mode_combo.setCurrentIndex(0)
        self.coord_stack.setCurrentIndex(0)
    
    def update_ui(self):
        """Обновление интерфейса."""
        # Обновление отображения состояния робота
        if self.robot.is_connected():
            # Обновление статуса подключения
            self.connection_label.setText("Подключен")
            self.connection_label.setStyleSheet("color: green;")
            
            # Обновление текущего положения
            position = self.robot.get_current_position()
            angles = self.robot.get_current_angles()
            
            # Отображение текущего положения в статусной строке
            pos_str = f"X: {position[0]:.2f}, Y: {position[1]:.2f}, Z: {position[2]:.2f}"
            self.status_label.setText(pos_str)
            
            # Обновление визуализации, если она доступна
            self.visualization.update_robot_model(position, angles)
        
        # Обновление логов
        self.update_log_display()
    
    def update_log_display(self):
        """Обновление отображения логов."""
        logs = self.logger.get_recent_logs()
        if logs and logs != self.log_text.toPlainText():
            self.log_text.clear()
            self.log_text.setText("\n".join(logs))
            # Прокрутка вниз
            self.log_text.moveCursor(self.log_text.textCursor().End)
    
    def connect_to_robot(self):
        """Подключение к роботу."""
        # Попытка подключения к роботу
        if self.robot.connect():
            # Обновление состояния кнопок
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            
            # Включение кнопок управления
            self.btn_move.setEnabled(True)
            self.btn_home.setEnabled(True)
            self.btn_grip.setEnabled(True)
            self.btn_release.setEnabled(True)
            self.btn_goto_preset.setEnabled(True)
            self.btn_start_auto.setEnabled(True)
            
            # Лог о подключении
            self.logger.info("Подключение к роботу установлено")
            
            # Сообщение в статусной строке
            self.connection_label.setText("Подключен")
            self.connection_label.setStyleSheet("color: green;")
        else:
            # Ошибка подключения
            self.logger.error("Не удалось подключиться к роботу")
            
            # Диалог с ошибкой
            QMessageBox.critical(self, "Ошибка подключения", 
                                 "Не удалось подключиться к роботу. Проверьте подключение и параметры.")
    
    def disconnect_from_robot(self):
        """Отключение от робота."""
        # Отключение от робота
        if self.robot.disconnect():
            # Обновление состояния кнопок
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            
            # Отключение кнопок управления
            self.btn_move.setEnabled(False)
            self.btn_home.setEnabled(False)
            self.btn_grip.setEnabled(False)
            self.btn_release.setEnabled(False)
            self.btn_goto_preset.setEnabled(False)
            self.btn_start_auto.setEnabled(False)
            
            # Остановка автоматического режима, если он запущен
            if self.btn_stop_auto.isEnabled():
                self.stop_auto_mode()
            
            # Лог об отключении
            self.logger.info("Отключение от робота выполнено")
            
            # Сообщение в статусной строке
            self.connection_label.setText("Не подключен")
            self.connection_label.setStyleSheet("color: red;")
        else:
            # Ошибка отключения
            self.logger.error("Ошибка при отключении от робота")
    
    def emergency_stop(self):
        """Аварийная остановка."""
        # Выполнение аварийной остановки
        if self.robot.emergency_stop():
            # Лог
            self.logger.warning("Выполнена аварийная остановка")
            
            # Остановка автоматического режима, если он запущен
            if self.btn_stop_auto.isEnabled():
                self.stop_auto_mode()
            
            # Диалог с предупреждением
            QMessageBox.warning(self, "Аварийная остановка", 
                                "Выполнена аварийная остановка. Робот остановлен.")
        else:
            # Ошибка
            self.logger.error("Ошибка при выполнении аварийной остановки")
    
    def on_coord_mode_changed(self, index):
        """Обработчик изменения режима координат."""
        # Переключение вкладки в зависимости от выбранного режима
        self.coord_stack.setCurrentIndex(index)
    
    def move_robot(self):
        """Перемещение робота."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Получение координат в зависимости от выбранного режима
        if self.coord_mode_combo.currentIndex() == 0:
            # Декартовы координаты
            try:
                x = float(self.edit_x.text())
                y = float(self.edit_y.text())
                z = float(self.edit_z.text())
                
                # Перемещение робота
                if self.robot.move_to_position(x, y, z):
                    self.logger.info(f"Перемещение на позицию X: {x}, Y: {y}, Z: {z}")
                else:
                    self.logger.error("Ошибка при перемещении робота")
            except ValueError:
                self.logger.error("Некорректный формат координат")
                QMessageBox.critical(self, "Ошибка", "Введите корректные числовые значения координат")
        else:
            # Углы сервоприводов
            try:
                angles = [float(angle_edit.text()) for angle_edit in self.angle_inputs]
                
                # Перемещение робота
                if self.robot.move_to_angles(angles):
                    self.logger.info(f"Перемещение на углы: {angles}")
                else:
                    self.logger.error("Ошибка при перемещении робота")
            except ValueError:
                self.logger.error("Некорректный формат углов")
                QMessageBox.critical(self, "Ошибка", "Введите корректные числовые значения углов")
    
    def home_position(self):
        """Перемещение в домашнюю позицию."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Перемещение в домашнюю позицию
        if self.robot.home_position():
            self.logger.info("Перемещение в домашнюю позицию")
        else:
            self.logger.error("Ошибка при перемещении в домашнюю позицию")
    
    def control_gripper(self, close):
        """Управление гриппером."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Управление гриппером
        action = "Захват" if close else "Разжим"
        if self.robot.set_gripper(close):
            self.logger.info(f"Выполнен {action}")
        else:
            self.logger.error(f"Ошибка при выполнении действия {action}")
    
    def goto_preset(self):
        """Переход к предустановленной позиции."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Получение выбранной позиции
        preset_index = self.preset_combo.currentIndex()
        preset_name = self.preset_combo.currentText()
        
        # Переход к выбранной позиции
        if self.robot.goto_preset(preset_index):
            self.logger.info(f"Переход к позиции {preset_name}")
        else:
            self.logger.error(f"Ошибка при переходе к позиции {preset_name}")
    
    def start_auto_mode(self):
        """Запуск автоматического режима."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Определение режима работы
        cycle_mode = self.chk_cycle.isChecked()
        mode_str = "циклический" if cycle_mode else "одиночный"
        
        # Запуск автоматического режима
        if self.robot.start_auto_mode(cycle_mode):
            self.logger.info(f"Запущен автоматический режим ({mode_str})")
            
            # Обновление кнопок
            self.btn_start_auto.setEnabled(False)
            self.btn_stop_auto.setEnabled(True)
            
            # Отключение других кнопок управления
            self.btn_move.setEnabled(False)
            self.btn_home.setEnabled(False)
            self.btn_grip.setEnabled(False)
            self.btn_release.setEnabled(False)
            self.btn_goto_preset.setEnabled(False)
        else:
            self.logger.error("Ошибка при запуске автоматического режима")
    
    def stop_auto_mode(self):
        """Остановка автоматического режима."""
        # Проверка подключения
        if not self.robot.is_connected():
            self.logger.error("Робот не подключен")
            return
        
        # Остановка автоматического режима
        if self.robot.stop_auto_mode():
            self.logger.info("Автоматический режим остановлен")
            
            # Обновление кнопок
            self.btn_start_auto.setEnabled(True)
            self.btn_stop_auto.setEnabled(False)
            
            # Включение других кнопок управления
            self.btn_move.setEnabled(True)
            self.btn_home.setEnabled(True)
            self.btn_grip.setEnabled(True)
            self.btn_release.setEnabled(True)
            self.btn_goto_preset.setEnabled(True)
        else:
            self.logger.error("Ошибка при остановке автоматического режима")
    
    def clear_logs(self):
        """Очистка логов."""
        self.logger.clear()
        self.log_text.clear()
        self.logger.info("Логи очищены")
    
    def save_logs(self):
        """Сохранение логов в файл."""
        # Открытие диалога сохранения файла
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Сохранение логов", "", "Текстовые файлы (*.txt);;Все файлы (*)",
            options=options
        )
        
        if file_path:
            # Сохранение логов в выбранный файл
            if self.logger.save_to_file(file_path):
                self.logger.info(f"Логи сохранены в файл: {file_path}")
            else:
                self.logger.error(f"Ошибка при сохранении логов в файл: {file_path}")
    
    def show_about_dialog(self):
        """Отображение диалога "О программе"."""
        QMessageBox.about(self, "О программе",
                         "Графический интерфейс для управления роботом-манипулятором\n\n"
                         "Версия: 1.0\n"
                         "Разработано в рамках конкурсного задания\n\n"
                         "© 2025")
    
    def closeEvent(self, event):
        """Обработчик закрытия окна."""
        # Отключаемся от робота при закрытии
        if self.robot.is_connected():
            self.robot.disconnect()
        
        # Остановка таймера
        self.update_timer.stop()
        
        # Лог о закрытии
        self.logger.info("Графический интерфейс закрыт")
        
        # Принятие события закрытия
        event.accept()


def main():
    """Точка входа в программу."""
    app = QApplication(sys.argv)
    
    # Запуск основного окна
    window = RobotGUI()
    
    # Запуск цикла обработки событий
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 