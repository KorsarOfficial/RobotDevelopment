import sys
import time
import requests
from io import BytesIO
from PyQt6.QtCore import QSize, Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QHBoxLayout,
    QSizePolicy,
    QMessageBox,
    QScrollArea,
)
from PyQt6.QtGui import QIcon, QTextCursor, QPixmap,QAction
import subprocess


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Пульт")
        self.setMinimumSize(800, 600)
        self.rotation_angle = 0
        self.button_press_start_time = None
        self.is_button_pressed = False
        self.current_button_text = None
        self.rotation_timer = QTimer()
        self.rotation_timer.timeout.connect(self.update_rotation)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # 1. Кнопки
        button_widget = QWidget()
        grid_layout = QGridLayout(button_widget)
        button_data = [
            (
                "https://images.icon-icons.com/1129/PNG/512/leftarrowangleincircularbutton_79880.png",
                "Влево",
                True,
                self.move_left,  # Обработчик кнопки
            ),
            (
                "https://images.icon-icons.com/1883/PNG/512/playbutton1_120626.png",
                "Вправо",
                True,
                self.move_right,  # Обработчик кнопки
            ),
            (
                "https://cdn-icons-png.flaticon.com/512/54/54293.png",
                "Вниз",
                True,
                self.move_down,  # Обработчик кнопки
            ),
            (
                "https://milton.ru/images/arrow_to_top.png",
                "Вверх",
                True,
                self.move_up,  # Обработчик кнопки
            ),
            ("Зажать", "Зажать", False, self.move_towards),  # Текстовые кнопки
            ("Разжать", "Разжать", False, self.move_away),
        ]

        positions = [(i, j) for i in range(2) for j in range(3)]

        for position, button_info in zip(positions, button_data):
            content, tooltip_text, is_image, handler = button_info
            button = QPushButton()

            if is_image:
                # Загружаем изображение из URL
                try:
                    response = requests.get(content)
                    response.raise_for_status()
                    image = QPixmap()
                    image.loadFromData(BytesIO(response.content).read())
                    icon = QIcon(image)
                    button.setIcon(icon)
                except requests.exceptions.RequestException as e:
                    print(f"Ошибка загрузки изображения: {e}")
                    button.setText("Ошибка")
            else:
                # Устанавливаем текст для текстовых кнопок
                button.setText(content)

            button.setIconSize(QSize(80, 80))
            button.setToolTip(tooltip_text)
            # Изменяем connect, чтобы вызывать нужный обработчик
            button.pressed.connect(
                lambda text=tooltip_text, h=handler: self.button_pressed(text, h)
            )
            button.released.connect(self.button_released)
            grid_layout.addWidget(button, position[0], position[1])

        button_widget.setLayout(grid_layout)
        main_layout.addWidget(button_widget)

        # 2. Консоль
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        console_height = int(self.height() * 0.3)
        self.console_output.setFixedHeight(console_height)
        main_layout.addWidget(self.console_output)

        # 3. Текстовое окно информации
        self.info_output = QTextEdit()
        self.info_output.setReadOnly(True)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.info_output)
        main_layout.addWidget(scroll_area)

        # Устанавливаем макет для central_widget
        central_widget.setLayout(main_layout)

        # 4. Кнопка Idle (в меню)
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("Файл")
        idle_action = QAction("Python Idle", self)
        idle_action.triggered.connect(self.open_idle) # не работает
        file_menu.addAction(idle_action)

        help_menu = menu_bar.addMenu("Помощь")
        about_action = QAction("Программа создана, как графический интерфейс для управления роботом", self)
        about_action.triggered.connect(self.show_about_dialog)
        help_menu.addAction(about_action)

        central_widget.setLayout(main_layout)

    def button_pressed(self, button_text, handler):
        self.current_button_text = button_text
        self.rotation_angle = 0
        self.button_press_start_time = time.time()
        self.is_button_pressed = True
        self.current_handler = handler # Сохраняем обработчик
        self.rotation_timer.start(100)
        self.update_info_output(button_text)
        self.current_handler() # Вызываем обработчик при нажатии


    def button_released(self):
        if self.is_button_pressed:
            self.rotation_timer.stop()
            self.is_button_pressed = False
            message = (
                f"Нажата кнопка: {self.current_button_text}\n"
                f"Угол поворота: {self.rotation_angle} градусов"
            )
            self.append_to_console(message)
            self.update_info_output(self.current_button_text)

    def update_rotation(self):
        if self.is_button_pressed:
            elapsed_time = time.time() - self.button_press_start_time
            self.rotation_angle = round(elapsed_time)
            self.update_info_output(self.current_button_text)

    def update_info_output(self, button_text):
        message = f"Нажата кнопка: {button_text}\nУгол поворота: {self.rotation_angle} градусов"
        self.info_output.setText(message)
        self.info_output.ensureCursorVisible()
        self.info_output.moveCursor(QTextCursor.MoveOperation.End)

    def append_to_console(self, message):
        self.console_output.append(message)
        self.console_output.ensureCursorVisible()
        self.console_output.moveCursor(QTextCursor.MoveOperation.End)

    def open_idle(self):
        try:
            subprocess.Popen(["idle"])
        except FileNotFoundError:
            QMessageBox.critical(self, "Ошибка", "IDLE не найден в системе.")

    def show_about_dialog(self):
        QMessageBox.information(self, "О программе", "Программа для управления направлением.")

    # Обработчики кнопок
    def move_left(self):
        # Дальше управление роботом
        print("Робот двигается влево")

    def move_right(self):
        # Дальше управление роботом
        print("Робот двигается вправо")

    def move_down(self):
        # Дальше управление роботом
        print("Робот двигается вниз")

    def move_up(self):
        # Дальше управление роботом
        print("Робот двигается вверх")

    def move_towards(self):
        # Дальше управление роботом
        print("Клешня зажимается")

    def move_away(self):
        # Дальше управление роботом
        print("Клешня разжимается")


app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()