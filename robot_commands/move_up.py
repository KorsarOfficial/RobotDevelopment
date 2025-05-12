#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Скрипт для поднятия руки робота RM 001.
"""

import sys
import getopt
from ctypes import *
import time

# Импортируем утилиты и константы
try:
    from utils import load_risdk_library, setup_sdk_functions, decode_error
    from constants import *
except ImportError:
    from robot_commands.utils import load_risdk_library, setup_sdk_functions, decode_error
    from robot_commands.constants import *

# Структура дескрипторов устройства
device = {
    "i2c": c_int(),        # дескриптор i2c
    "pwm": c_int(),        # дескриптор pwm
    "body": c_int(),       # дескриптор сервопривода тела
    "claw": c_int(),       # дескриптор сервопривода клешни
    "arrowR": c_int(),     # дескриптор сервопривода правой стрелы
    "arrowL": c_int(),     # дескриптор сервопривода левой стрелы
    "clawRotate": c_int(), # дескриптор сервопривода поворота клешни
    "led": c_int(),        # дескриптор светодиода
}

# Массив дескрипторов сервоприводов и их стартовых позиций
servos = [
    {"descriptor": device["body"], "start_position": BODY_START_PULSE},
    {"descriptor": device["claw"], "start_position": CLAW_START_PULSE},
    {"descriptor": device["arrowR"], "start_position": ARROW_R_START_PULSE},
    {"descriptor": device["arrowL"], "start_position": ARROW_L_START_PULSE},
    {"descriptor": device["clawRotate"], "start_position": CLAW_ROTATE_START_PULSE},
]

def init_servos():
    """
    Создает сервоприводы и линкует их.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = c_int

    # Создаем 5 сервоприводов и линкуем их к пинам 0-4
    for i in range(len(servos)):
        # Создаем компонент сервопривода с конкретной моделью как исполняемое устройство
        errCode = lib.RI_SDK_CreateModelComponent(
            "executor".encode(), 
            "servodrive".encode(), 
            "mg90s".encode(),
            servos[i]["descriptor"], 
            errTextC
        )
        if errCode != 0:
            return errCode, errTextC
            
        # Связываем сервопривод с ШИМ
        errCode = lib.RI_SDK_LinkServodriveToController(
            servos[i]["descriptor"], 
            device["pwm"], 
            i, 
            errTextC
        )
        if errCode != 0:
            return errCode, errTextC
            
    return errCode, errTextC

def init_device():
    """
    Инициализация библиотеки и устройств.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = c_int
    i2c_name = "cp2112"  # По умолчанию используем cp2112

    # Получаем имя драйвера i2c из аргументов командной строки
    try:
        opts, args = getopt.getopt(sys.argv[1:], "d:", [])
        for opt, arg in opts:
            if opt in ("-d"):
                i2c_name = arg
    except getopt.GetoptError:
        print("Использование: move_up.py -d <i2c_adapter_name>")
        sys.exit(2)

    # Вызываем функцию инициализации SDK
    errCode = lib.RI_SDK_InitSDK(LOG_LEVEL, errTextC)
    if errCode != 0:
        return errCode, errTextC

    # Создаем компонент ШИМ
    errCode = lib.RI_SDK_CreateModelComponent(
        "connector".encode(), 
        "pwm".encode(), 
        "pca9685".encode(), 
        device["pwm"],
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    # Создаем компонент i2c адаптера
    errCode = lib.RI_SDK_CreateModelComponent(
        "connector".encode(), 
        "i2c_adapter".encode(), 
        i2c_name.encode(),
        device["i2c"], 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    # Связываем i2c адаптер с ШИМ по адресу 0x40
    errCode = lib.RI_SDK_LinkPWMToController(
        device["pwm"], 
        device["i2c"], 
        c_uint8(0x40), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    # Создаем компонент светодиода
    errCode = lib.RI_SDK_CreateModelComponent(
        "executor".encode(), 
        "led".encode(), 
        "ky016".encode(), 
        device["led"],
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    # Связываем светодиод с ШИМ
    errCode = lib.RI_SDK_LinkLedToController(
        device["led"], 
        device["pwm"], 
        15, 14, 13, 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    # Инициализируем сервоприводы
    errCode, errTextC = init_servos()
    if errCode != 0:
        return errCode, errTextC

    return errCode, errTextC

def start_position(servo):
    """
    Переводит сервопривод в стартовое положение.
    
    Args:
        servo (dict): Словарь с дескриптором сервопривода и стартовой позицией.
        
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    
    # Выполняем поворот сервопривода в стартовую позицию
    errCode = lib.RI_SDK_exec_ServoDrive_TurnByPulse(
        servo["descriptor"], 
        servo["start_position"], 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC

    time.sleep(0.5)  # Пауза для стабилизации
    
    return errCode, errTextC

def start_position_all_servo():
    """
    Переводит все сервоприводы в стартовую позицию.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    
    # Включаем красный светодиод (начало движения)
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(
        device["led"], 
        255, 0, 0,  # RGB: красный 
        0, 
        c_bool(True), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
        
    # Приводим сервоприводы в стартовое положение
    for servo in servos:
        errCode, errTextC = start_position(servo)
        if errCode != 0:
            return errCode, errTextC
    
    # Включаем зеленый светодиод (успешное завершение)
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(
        device["led"], 
        0, 255, 0,  # RGB: зеленый
        0, 
        c_bool(True), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
        
    time.sleep(0.5)  # Пауза для стабилизации
    
    return errCode, errTextC

def move_up():
    """
    Поднимает руку робота.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    
    # Включаем мигание пурпурным светодиодом при движении
    errCode = lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency(
        device["led"], 
        128, 0, 128,  # RGB: пурпурный
        5,  # частота мигания
        0, 
        c_bool(True), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
    
    # Поднимаем правую стрелу (arrowR)
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(
        device["arrowR"], 
        MOVE_UP_ANGLE_R, 
        SERVO_SPEED, 
        c_bool(False),  # не асинхронно
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
    
    # Поднимаем левую стрелу (arrowL)
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(
        device["arrowL"], 
        MOVE_UP_ANGLE_L, 
        SERVO_SPEED, 
        c_bool(False),  # не асинхронно
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
    
    # Включаем зеленый светодиод (успешное завершение)
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(
        device["led"], 
        0, 255, 0,  # RGB: зеленый
        0, 
        c_bool(True), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
    
    return errCode, errTextC

def destruct_servos():
    """
    Уничтожает сервоприводы.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = c_int
    
    # Уничтожаем все сервоприводы
    for servo in servos:
        errCode = lib.RI_SDK_DestroyComponent(servo["descriptor"], errTextC)
        if errCode != 0:
            return errCode, errTextC
    
    return errCode, errTextC

def destruct():
    """
    Уничтожает все компоненты и библиотеку.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    
    # Включаем красный светодиод (завершение работы)
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(
        device["led"], 
        255, 0, 0,  # RGB: красный
        0, 
        c_bool(True), 
        errTextC
    )
    if errCode != 0:
        return errCode, errTextC
    
    # Уничтожаем сервоприводы
    errCode, errTextC = destruct_servos()
    if errCode != 0:
        return errCode, errTextC
    
    # Останавливаем светодиод
    errCode = lib.RI_SDK_exec_RGB_LED_Stop(device["led"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    # Уничтожаем светодиод
    errCode = lib.RI_SDK_DestroyComponent(device["led"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    # Сбрасываем все порты ШИМ
    errCode = lib.RI_SDK_sigmod_PWM_ResetAll(device["pwm"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    # Уничтожаем ШИМ
    errCode = lib.RI_SDK_DestroyComponent(device["pwm"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    # Уничтожаем i2c
    errCode = lib.RI_SDK_DestroyComponent(device["i2c"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    # Уничтожаем SDK
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    
    return errCode, errTextC

def start():
    """
    Запускает программу поднятия руки.
    
    Returns:
        tuple: (errCode, errTextC) - код ошибки и текст ошибки.
    """
    # Инициализируем библиотеку и компоненты
    errCode, errText = init_device()
    if errCode != 0:
        return errCode, errText
    
    # Приводим сервоприводы к стартовой позиции
    errCode, errText = start_position_all_servo()
    if errCode != 0:
        return errCode, errText
    
    # Поднимаем руку робота
    errCode, errText = move_up()
    if errCode != 0:
        return errCode, errText
    
    # Задержка для наблюдения за результатом
    time.sleep(2)
    
    # Уничтожаем компоненты и библиотеку
    errCode, errText = destruct()
    if errCode != 0:
        return errCode, errText
    
    return errCode, errText

def main():
    """
    Главная функция запускающая всё остальное.
    """
    print("Запуск робота RM 001 - поднятие руки")
    
    errCode, errText = start()
    if errCode != 0:
        print(f"Ошибка: {errCode}, {decode_error(errText)}")
        sys.exit(2)
    
    print("Робот успешно поднял руку")

# Загружаем библиотеку RISDK
lib = load_risdk_library()
setup_sdk_functions(lib)

if __name__ == "__main__":
    main() 