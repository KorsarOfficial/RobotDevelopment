#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Пример вращения робота RM 001 вокруг оси X
"""

import os
import sys
import time
import argparse
import logging

# Импортируем конфигурацию, утилиты и контроллер робота
try:
    from config import *
    from utils import setup_logger
    from robot_controller import RobotController
except ImportError:
    # Если файлы не найдены в текущей директории
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from number4.config import *
    from number4.utils import setup_logger
    from number4.robot_controller import RobotController

# Инициализация логирования
logger = setup_logger()

def parse_arguments():
    """
    Разбор аргументов командной строки.
    
    Returns:
        argparse.Namespace: Аргументы командной строки.
    """
    parser = argparse.ArgumentParser(description="Пример вращения робота RM 001 вокруг оси X")
    parser.add_argument("--emulation", action="store_true", help="Включить режим эмуляции робота")
    parser.add_argument("--cycles", type=int, default=1, help="Количество циклов вращения")
    
    return parser.parse_args()

def display_welcome_message():
    """Вывод приветственного сообщения."""
    print("\n" + "="*60)
    print("Пример №4: Вращение робота RM 001 вокруг оси X".center(60))
    print("="*60 + "\n")
    print("Этот пример демонстрирует вращение робота вокруг оси X с помощью сервопривода плеча.")
    print("Робот будет совершать поворот от {} до {} градусов и обратно.".format(
        X_ROTATION_PARAMS["min_angle"], X_ROTATION_PARAMS["max_angle"]))
    print("\nВ этом примере используется робот-манипулятор RM 001.")
    print("\n" + "-"*60 + "\n")

def get_user_choice():
    """
    Получение выбора пользователя.
    
    Returns:
        str: Выбор пользователя.
    """
    print(f"Введите {USER_CHOICE_START} и нажмите Enter, чтобы начать вращение робота.")
    print(f"Введите {USER_CHOICE_EXIT} и нажмите Enter, чтобы выйти из программы.")
    
    # Читаем ввод пользователя
    choice = input("Ваш выбор: ")
    return choice

def run_x_rotation_demo(args):
    """
    Запуск демонстрации вращения вокруг оси X.
    
    Args:
        args (argparse.Namespace): Аргументы командной строки.
    """
    # Вывод приветственного сообщения
    display_welcome_message()
    
    # Установка режима эмуляции
    emulation_mode = args.emulation or EMULATION_MODE
    
    try:
        # Создание экземпляра контроллера робота
        logger.info("Создание контроллера робота")
        robot = RobotController(emulation=emulation_mode)
        
        while True:
            # Получение выбора пользователя
            choice = get_user_choice()
            
            if choice == USER_CHOICE_START:
                logger.info("Запуск последовательности вращения вокруг оси X")
                print("\nЗапуск последовательности вращения вокруг оси X...\n")
                
                # Запуск последовательности вращения
                result = robot.perform_x_rotation_sequence(cycles=args.cycles)
                
                if result:
                    print("\nПоследовательность вращения успешно завершена.\n")
                else:
                    print("\nПроизошла ошибка при выполнении последовательности вращения.\n")
            
            elif choice == USER_CHOICE_EXIT:
                print("\nЗавершение программы...\n")
                break
            
            else:
                print("\nНеверный выбор. Пожалуйста, попробуйте снова.\n")
    
    except Exception as e:
        logger.error("Произошла ошибка: %s", str(e))
        print(f"\nПроизошла ошибка: {str(e)}\n")
    
    finally:
        # Корректное завершение работы с роботом
        if 'robot' in locals():
            robot.disconnect()
        
        logger.info("Программа завершена")
        print("\nПрограмма завершена.\n")

def main():
    """Основная функция программы."""
    try:
        # Разбор аргументов командной строки
        args = parse_arguments()
        
        # Запуск демонстрации
        run_x_rotation_demo(args)
        
    except KeyboardInterrupt:
        logger.info("Программа прервана пользователем")
        print("\nПрограмма прервана пользователем.\n")
    
    except Exception as e:
        logger.error("Необработанная ошибка: %s", str(e))
        print(f"\nПроизошла необработанная ошибка: {str(e)}\n")

if __name__ == "__main__":
    main() 