#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import platform
from ctypes import *
import time

# Импортируем константы
try:
    from constants import *
except ImportError:
    from robot_commands.constants import *

def load_risdk_library():
    """
    Загружает библиотеку RISDK в зависимости от платформы.
    
    Returns:
        ctypes.CDLL: Загруженная библиотека.
    """
    current_platform = platform.system()
    if current_platform == PLATFORM_WINDOWS:
        return cdll.LoadLibrary("./librisdk.dll")
    elif current_platform == PLATFORM_LINUX:
        return cdll.LoadLibrary("./librisdk.so")
    else:
        print(f"Неподдерживаемая платформа: {current_platform}")
        sys.exit(1)

def setup_sdk_functions(lib):
    """
    Настраивает типы аргументов для функций библиотеки RISDK.
    
    Args:
        lib (ctypes.CDLL): Библиотека RISDK.
    """
    # Инициализация SDK
    lib.RI_SDK_InitSDK.argtypes = [c_int, c_char_p]
    
    # Создание компонентов
    lib.RI_SDK_CreateModelComponent.argtypes = [c_char_p, c_char_p, c_char_p, POINTER(c_int), c_char_p]
    
    # Связывание компонентов
    lib.RI_SDK_LinkPWMToController.argtypes = [c_int, c_int, c_uint8, c_char_p]
    lib.RI_SDK_LinkLedToController.argtypes = [c_int, c_int, c_int, c_int, c_int, c_char_p]
    lib.RI_SDK_LinkServodriveToController.argtypes = [c_int, c_int, c_int, c_char_p]
    
    # Управление LED
    lib.RI_SDK_exec_RGB_LED_SinglePulse.argtypes = [c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
    lib.RI_SDK_exec_RGB_LED_Stop.argtypes = [c_int, c_char_p]
    lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
    lib.RI_SDK_exec_RGB_LED_Flicker.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
    
    # Управление сервоприводами
    lib.RI_SDK_exec_ServoDrive_TurnByPulse.argtypes = [c_int, c_int, c_char_p]
    lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
    
    # Управление ШИМ
    lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
    
    # Удаление компонентов
    lib.RI_SDK_DestroyComponent.argtypes = [c_int, c_char_p]
    lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]

def decode_error(err):
    """
    Декодирует текст ошибки из C-строки.
    
    Args:
        err (ctypes.c_char_p): Текст ошибки в формате C-строки.
        
    Returns:
        str: Декодированный текст ошибки.
    """
    return err.raw.decode()

def print_command_help():
    """
    Выводит справку по командам интерактивного режима.
    """
    print("\nДоступные команды:")
    print(f"  {CMD_LEFT} - повернуть робота влево")
    print(f"  {CMD_RIGHT} - повернуть робота вправо")
    print(f"  {CMD_UP} - поднять руку робота")
    print(f"  {CMD_DOWN} - опустить руку робота")
    print(f"  {CMD_OPEN} - открыть клешню")
    print(f"  {CMD_CLOSE} - закрыть клешню")
    print(f"  {CMD_HOME} - вернуть робота в домашнюю позицию")
    print(f"  {CMD_HELP} - показать эту справку")
    print(f"  {CMD_EXIT} - выйти из программы\n") 