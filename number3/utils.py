#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Вспомогательные функции для работы с камерой и обработки изображений
"""

import os
import sys
import time
import logging
import cv2
import numpy as np
from pathlib import Path

# Импортируем конфигурацию
try:
    from config import *
except ImportError:
    # Если конфигурация не найдена в текущей директории
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from number3.config import *


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
    logger = logging.getLogger("camera_example")
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


def get_camera_device():
    """
    Получение устройства камеры
    
    Returns:
        tuple: (success, camera_id)
    """
    if CAMERA_TYPE == "usb":
        return True, int(CAMERA_DEVICE_ID)
    elif CAMERA_TYPE == "ip":
        return True, CAMERA_URL
    else:
        return False, None


def get_aruco_dictionary():
    """
    Получение словаря ArUco из заданного в конфигурации
    
    Returns:
        cv2.aruco.Dictionary: Словарь ArUco
    """
    aruco_dict_map = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }
    
    if ARUCO_DICTIONARY in aruco_dict_map:
        return cv2.aruco.Dictionary_get(aruco_dict_map[ARUCO_DICTIONARY])
    else:
        # По умолчанию используем 4x4_50
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def get_aruco_parameters():
    """
    Получение параметров детектирования ArUco
    
    Returns:
        cv2.aruco.DetectorParameters: Параметры детектирования
    """
    parameters = cv2.aruco.DetectorParameters_create()
    
    # Устанавливаем параметры из конфигурации
    for param_name, param_value in ARUCO_DETECTION_PARAMS.items():
        if hasattr(parameters, param_name):
            setattr(parameters, param_name, param_value)
    
    return parameters


def load_camera_calibration():
    """
    Загрузка данных калибровки камеры
    
    Returns:
        tuple: (camera_matrix, dist_coeffs, success)
    """
    try:
        if os.path.exists(CAMERA_CALIBRATION_FILE):
            calibration_data = np.load(CAMERA_CALIBRATION_FILE)
            camera_matrix = calibration_data['camera_matrix']
            dist_coeffs = calibration_data['dist_coeffs']
            return camera_matrix, dist_coeffs, True
        else:
            # Если файл не найден, возвращаем примерные значения
            camera_matrix = np.array([
                [CAMERA_WIDTH / 2, 0, CAMERA_WIDTH / 2],
                [0, CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros((5, 1))
            return camera_matrix, dist_coeffs, False
    except Exception as e:
        # В случае ошибки возвращаем примерные значения
        camera_matrix = np.array([
            [CAMERA_WIDTH / 2, 0, CAMERA_WIDTH / 2],
            [0, CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2],
            [0, 0, 1]
        ])
        dist_coeffs = np.zeros((5, 1))
        return camera_matrix, dist_coeffs, False


def save_camera_calibration(camera_matrix, dist_coeffs):
    """
    Сохранение данных калибровки камеры
    
    Args:
        camera_matrix (numpy.ndarray): Матрица камеры
        dist_coeffs (numpy.ndarray): Коэффициенты дисторсии
    
    Returns:
        bool: Успешно ли сохранение
    """
    try:
        # Создаем директорию для файла калибровки, если ее нет
        calib_dir = os.path.dirname(CAMERA_CALIBRATION_FILE)
        if calib_dir and not os.path.exists(calib_dir):
            os.makedirs(calib_dir)
        
        # Сохраняем данные
        np.savez(CAMERA_CALIBRATION_FILE, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        return True
    except Exception as e:
        return False


def detect_chessboard(image):
    """
    Обнаружение углов шахматной доски на изображении
    
    Args:
        image (numpy.ndarray): Изображение
    
    Returns:
        tuple: (success, corners, image_with_corners)
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    
    if ret:
        # Уточнение положения углов
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Рисуем углы на изображении
        image_with_corners = image.copy()
        cv2.drawChessboardCorners(image_with_corners, CHESSBOARD_SIZE, corners, ret)
        return True, corners, image_with_corners
    else:
        return False, None, image


def create_calibration_object_points():
    """
    Создание точек объекта для калибровки
    
    Returns:
        numpy.ndarray: Массив точек объекта
    """
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp = objp * CHESSBOARD_SQUARE_SIZE
    return objp


def calibrate_camera(obj_points, img_points):
    """
    Калибровка камеры
    
    Args:
        obj_points (list): Список точек объекта
        img_points (list): Список точек изображения
    
    Returns:
        tuple: (success, camera_matrix, dist_coeffs, rvecs, tvecs)
    """
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (CAMERA_WIDTH, CAMERA_HEIGHT), None, None
    )
    return ret, camera_matrix, dist_coeffs, rvecs, tvecs


def preprocess_image(image):
    """
    Предварительная обработка изображения
    
    Args:
        image (numpy.ndarray): Исходное изображение
    
    Returns:
        numpy.ndarray: Обработанное изображение
    """
    # Конвертируем в оттенки серого
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Повышаем контраст
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    
    # Немного сглаживаем для уменьшения шума
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    return blur


def detect_aruco_markers(image, camera_matrix=None, dist_coeffs=None):
    """
    Обнаружение ArUco маркеров на изображении
    
    Args:
        image (numpy.ndarray): Изображение
        camera_matrix (numpy.ndarray, optional): Матрица камеры
        dist_coeffs (numpy.ndarray, optional): Коэффициенты дисторсии
    
    Returns:
        tuple: (corners, ids, rejected, rvecs, tvecs)
    """
    # Получаем словарь и параметры ArUco
    aruco_dict = get_aruco_dictionary()
    parameters = get_aruco_parameters()
    
    # Обнаруживаем маркеры
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    
    # Если найдены маркеры и предоставлены параметры калибровки, оцениваем позу
    rvecs, tvecs = None, None
    if ids is not None and camera_matrix is not None and dist_coeffs is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, ARUCO_MARKER_SIZE, camera_matrix, dist_coeffs
        )
    
    return corners, ids, rejected, rvecs, tvecs


def draw_detected_markers(image, corners, ids, rvecs=None, tvecs=None, camera_matrix=None, dist_coeffs=None):
    """
    Рисование обнаруженных маркеров на изображении
    
    Args:
        image (numpy.ndarray): Изображение
        corners (list): Углы маркеров
        ids (numpy.ndarray): Идентификаторы маркеров
        rvecs (numpy.ndarray, optional): Векторы вращения
        tvecs (numpy.ndarray, optional): Векторы перемещения
        camera_matrix (numpy.ndarray, optional): Матрица камеры
        dist_coeffs (numpy.ndarray, optional): Коэффициенты дисторсии
    
    Returns:
        numpy.ndarray: Изображение с отрисованными маркерами
    """
    output_image = image.copy()
    
    # Рисуем обнаруженные маркеры
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(output_image, corners, ids)
        
        # Если доступны векторы позы, рисуем оси координат
        if rvecs is not None and tvecs is not None and camera_matrix is not None and dist_coeffs is not None:
            for i in range(len(ids)):
                cv2.aruco.drawAxis(output_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 50)
    
    return output_image


def calculate_marker_distance(tvec):
    """
    Расчет расстояния до маркера
    
    Args:
        tvec (numpy.ndarray): Вектор перемещения маркера
    
    Returns:
        float: Расстояние в миллиметрах
    """
    return np.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)


def calculate_marker_position_relative_to_robot(tvec, rvec, camera_to_robot_transform=None):
    """
    Расчет позиции маркера относительно робота
    
    Args:
        tvec (numpy.ndarray): Вектор перемещения маркера
        rvec (numpy.ndarray): Вектор вращения маркера
        camera_to_robot_transform (numpy.ndarray, optional): Матрица преобразования из системы 
                                                        координат камеры в систему координат робота
    
    Returns:
        numpy.ndarray: Позиция маркера в системе координат робота
    """
    # Если преобразование не предоставлено, используем единичное преобразование
    if camera_to_robot_transform is None:
        camera_to_robot_transform = np.eye(4)
    
    # Создаем матрицу преобразования камера-маркер
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    camera_to_marker = np.eye(4)
    camera_to_marker[:3, :3] = rotation_matrix
    camera_to_marker[:3, 3] = tvec.reshape(3)
    
    # Преобразуем в систему координат робота
    robot_to_marker = np.dot(camera_to_robot_transform, camera_to_marker)
    
    return robot_to_marker[:3, 3]


def save_image(image, directory="images", filename=None):
    """
    Сохранение изображения
    
    Args:
        image (numpy.ndarray): Изображение
        directory (str): Директория для сохранения
        filename (str, optional): Имя файла
    
    Returns:
        str: Путь к сохраненному файлу
    """
    # Создаем директорию, если ее нет
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    # Если имя файла не указано, генерируем его
    if filename is None:
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = f"image_{timestamp}.jpg"
    
    # Сохраняем изображение
    filepath = os.path.join(directory, filename)
    cv2.imwrite(filepath, image)
    
    return filepath


def pnp_from_marker(corners, marker_size, camera_matrix, dist_coeffs):
    """
    Решение задачи PnP для маркера
    
    Args:
        corners (numpy.ndarray): Углы маркера
        marker_size (float): Размер маркера
        camera_matrix (numpy.ndarray): Матрица камеры
        dist_coeffs (numpy.ndarray): Коэффициенты дисторсии
    
    Returns:
        tuple: (success, rvec, tvec)
    """
    # Создаем 3D точки маркера
    object_points = np.array([
        [-marker_size/2, marker_size/2, 0],
        [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ])
    
    # Получаем 2D точки из углов
    image_points = corners.reshape(4, 2)
    
    # Решаем PnP
    ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    
    return ret, rvec, tvec 