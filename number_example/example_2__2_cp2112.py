import sys
from ctypes import *

# Подключение библиотеки ri_sdk.
lib = cdll.LoadLibrary("./librisdk.dll")
# Объявление аргументов вызываемых функций.
lib.RI_SDK_InitSDK.argtypes = [c_int, c_char_p]
lib.RI_SDK_CreateModelComponent.argtypes = [c_char_p, c_char_p, c_char_p, POINTER(c_int), c_char_p]
lib.RI_SDK_LinkPWMToController.argtypes = [c_int, c_int, c_uint8, c_char_p]
lib.RI_SDK_LinkServodriveToController.argtypes = [c_int, c_int, c_int, c_char_p]
lib.RI_SDK_DestroyComponent.argtypes = [c_int, c_char_p]
lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
# Объявление переменных для
errTextC = create_string_buffer(1000)   # Перменная, в которую будет записываться текст ошибки
errCode = c_int() # Переменная, в которую будет записываться код ошибки
i2c = c_int() # Переменная, в которой будет храниться дескриптор на объект i2c адаптера
pwm = c_int() # Переменная, в которой будет храниться дескриптор на объект pwm модулятора
servo_1 = c_int() # Переменная, в которой будет храниться дескриптор на объект управления первым сервоприводом

def initComponents():
    # Инициализация RI SDK.
    errCode = lib.RI_SDK_InitSDK(2, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    # Инициализация i2c адаптера.
    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "i2c_adapter".encode(), "cp2112".encode(), i2c, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    # Инициализация ШИМ.
    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "pwm".encode(), "pca9685".encode(), pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Связывание ШИМ с i2c адаптером.
    errCode = lib.RI_SDK_LinkPWMToController(pwm, i2c, c_uint8(0x40), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    # Инициализация сервопривода.
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(), servo_1, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    # Связывание сервопривода с ШИМ.
    errCode = lib.RI_SDK_LinkServodriveToController(servo_1, pwm, 0, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)

def destroyComponentsAndSDK():
    errCode = lib.RI_SDK_sigmod_PWM_ResetAll(pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента сервопривода.
    errCode = lib.RI_SDK_DestroyComponent(servo_1, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента ШИМ.
    errCode = lib.RI_SDK_DestroyComponent(pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента i2c.
    errCode = lib.RI_SDK_DestroyComponent(i2c, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Завершение работы с RI SDK.
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)

def turnServodrive(descriptor, angle, speed):
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(descriptor, angle, speed, c_bool(False), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)

# main() - функция-"точка входа" в программах на Python и многих других языках.
def main():
    # Инициализация RI SDK.
    initComponents()
    # Поворот сервопривода на 90 градусов.
    turnServodrive(servo_1, 90, 100)
    # Поворот сервопривода на 90 градусов в противоположную сторону.
    turnServodrive(servo_1, -90, 100)
    # Поворот сервопривода на 90 градусов.
    turnServodrive(servo_1, 90, 100)
    # Поворот сервопривода на 90 градусов в противоположную сторону.
    turnServodrive(servo_1, -90, 100)
    # Поворот сервопривода на 90 градусов.
    turnServodrive(servo_1, 90, 100)
    # Сброс сигнала со всех портов ШИМ.
    errCode = lib.RI_SDK_sigmod_PWM_ResetAll(pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента сервопривода.
    errCode = lib.RI_SDK_DestroyComponent(servo_1, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента ШИМ.
    errCode = lib.RI_SDK_DestroyComponent(pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Удаление из памяти компонента i2c.
    errCode = lib.RI_SDK_DestroyComponent(i2c, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    # Завершение работы с RI SDK.
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)

main()