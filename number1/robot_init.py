#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Скрипт для инициализации и проверки подключения к роботу.
"""

import sys
import time
import logging
from robot_controller import RobotController

def main():
    """Основная функция проверки подключения к роботу."""
    print("="*50)
    print("Тестирование подключения к роботу-манипулятору")
    print("="*50)
    
    # Создаем экземпляр контроллера робота
    controller = RobotController()
    
    # Шаг 1: Инициализация SDK
    print("\nШаг 1: Инициализация RISDK...")
    if not controller.sdk:
        controller.sdk = controller.risdk.RI_SDK_InitSDK()
        if not controller.sdk:
            print("❌ Ошибка: Не удалось инициализировать RISDK")
            return
    print("✅ RISDK инициализирован успешно")
    
    # Шаг 2: Поиск доступных моделей роботов
    print("\nШаг 2: Поиск доступных моделей роботов...")
    available_models = controller.list_available_models()
    if not available_models:
        print("❌ Ошибка: Не найдено доступных моделей роботов")
        return
    
    print(f"✅ Найдено {len(available_models)} моделей роботов:")
    for model in available_models:
        print(f"   - {model}")
    
    # Шаг 3: Подключение к роботу
    print("\nШаг 3: Подключение к роботу...")
    model_name = "RM002"  # Можно изменить на другую модель из списка
    print(f"   Выбрана модель: {model_name}")
    
    if not controller.initialize(model_name):
        print(f"❌ Ошибка: Не удалось подключиться к роботу {model_name}")
        return
    
    print(f"✅ Успешное подключение к роботу {model_name}")
    
    # Шаг 4: Проверка доступных сервоприводов
    print("\nШаг 4: Проверка доступных сервоприводов...")
    servo_count = controller.get_servo_count()
    print(f"✅ Доступные сервоприводы: {servo_count}")
    
    # Шаг 5: Проверка статуса системы
    print("\nШаг 5: Проверка статуса системы...")
    status = controller.get_status()
    print(f"✅ Текущее состояние системы: {status}")
    
    # Шаг 6: Тестовое перемещение (опционально)
    print("\nШаг 6: Тестовое перемещение (опционально)...")
    response = input("Выполнить тестовое перемещение? (y/n): ")
    
    if response.lower() == 'y':
        # Создаем список углов для каждого сервопривода
        test_angles = [0.0] * servo_count
        
        # Запрашиваем углы у пользователя
        for i in range(servo_count):
            try:
                angle = float(input(f"Введите угол для сервопривода {i+1} (0-180): "))
                test_angles[i] = angle
            except ValueError:
                print(f"Ошибка ввода. Используется значение по умолчанию: 0.0")
        
        print(f"Выполняется перемещение с углами: {test_angles}")
        if controller.move_to_position(test_angles):
            print("✅ Перемещение выполнено успешно")
        else:
            print("❌ Ошибка при выполнении перемещения")
    else:
        print("Тестовое перемещение пропущено")
    
    # Шаг 7: Завершение работы
    print("\nШаг 7: Завершение работы...")
    controller.shutdown()
    print("✅ Работа с роботом завершена")
    
    print("\n" + "="*50)
    print("Тестирование подключения завершено")
    print("="*50)


if __name__ == "__main__":
    main() 