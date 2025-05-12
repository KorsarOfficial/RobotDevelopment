import sys,getopt
from ctypes import *
import platform
import time

# Подключаем внешнюю библиотеку для работы с SDK
platform = platform.system()
if platform == "Windows":
    lib = cdll.LoadLibrary("./librisdk.dll")
if platform == "Linux":
    lib = cdll.LoadLibrary("./librisdk.so")

# Инициализируем глобальные перменные
body_start_pulse = 1500  # стартовая позиция тела
arrowR_start_pulse = 2000  # стартовая возиция правой стрелы
arrowL_start_pulse = 1000  # стартовая позиция левой стрелы
claw_start_pulse = 1000  # стартовая позиция клешни
claw_rotate_start_pulse = 1500  # стартовая позиция поворота клешни

arrowR_over_cube_position = -30  # позиция правой стрелы над кубиком
arrowL_over_cube_position = -30  # позиция левой стрелы над кубиком
claw_unclenched_position = 100  # позиция открытой клешни
arrowR_cube_position = -65  # позиция правой стрелы на месте кубика
arrowL_cube_position = 35  # позиция левой стрелы на месте кубика

logLevel = 2  # уровень логирования
speed = 100  # скорость в градусах в секунду

# структура дескрипторов устройства: ключ-имя - значение дескриптора
device = {
    "i2c": c_int(),  # дескриптор i2c
    "pwm": c_int(),  # дескриптор pwm
    "body": c_int(),  # дескриптор сервопривода тела
    "claw": c_int(),  # дескриптор сервопривода клешни
    "arrowR": c_int(),  # дескриптор сервопривода правой стрелы
    "arrowL": c_int(),  # дескриптор сервопривода левой стрелы
    "clawRotate": c_int(),  # дескриптор сервопривода поворота клешни
    "led": c_int(),  # дескриптор светодиода
}

# массив дескрипторов сервоприводов и их стартовых позиций
servos = [
    {"descriptor": device["body"],
     "start_position": body_start_pulse
     },
    {"descriptor": device["claw"],
     "start_position": claw_start_pulse
     },
    {"descriptor": device["arrowR"],
     "start_position": arrowR_start_pulse
     },
    {"descriptor": device["arrowL"],
     "start_position": arrowL_start_pulse
     },
    {"descriptor": device["clawRotate"],
     "start_position": claw_rotate_start_pulse
     },
]

# Указываем типы аргументов для функций
lib.RI_SDK_InitSDK.argtypes = [c_int, c_char_p]
lib.RI_SDK_CreateModelComponent.argtypes = [c_char_p, c_char_p, c_char_p, POINTER(c_int), c_char_p]
lib.RI_SDK_LinkPWMToController.argtypes = [c_int, c_int, c_uint8, c_char_p]
lib.RI_SDK_LinkLedToController.argtypes = [c_int, c_int, c_int, c_int, c_int, c_char_p]
lib.RI_SDK_LinkServodriveToController.argtypes = [c_int, c_int, c_int, c_char_p]
lib.RI_SDK_exec_RGB_LED_SinglePulse.argtypes = [c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_TurnByPulse.argtypes = [c_int, c_int, c_char_p]
lib.RI_SDK_DestroyComponent.argtypes = [c_int, c_char_p]
lib.RI_SDK_exec_RGB_LED_Stop.argtypes = [c_int, c_char_p]
lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]
lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool,
                                                          c_char_p]
lib.RI_SDK_exec_RGB_LED_Flicker.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]


# для получения текста ошибки
def decodeErr(err):
    return err.raw.decode()


# initServos - создает сервоприводы и линкует их
def initServos():

    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = c_int
    # создаем 5 сервоприводов и линкуем их к пинам 0-4
    for i in range(len(servos)):
        # создаем компонент сервопривода с конкретной моделью как исполняемое устройство и получаем дескриптор сервопривода
        errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(),
                                                  servos[i]["descriptor"], errTextC)
        if errCode != 0:
            return errCode, errTextC
        # связываем сервопривод с ШИМ,передаем дескриптор сервопривода и ШИМ
        errCode = lib.RI_SDK_LinkServodriveToController(servos[i]["descriptor"], device["pwm"], i, errTextC)
        if errCode != 0:
            return errCode, errTextC
    return errCode, errTextC


# initDevice - инициализация библиотеки и устройств
def initDevice():
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = c_int
    i2c_name = str

    #Получаем имя драйвера i2c из аргументов командной строки
    opts, args = getopt.getopt(sys.argv[1:], "d:", [])
    for opt, arg in opts:
        if opt in ("-d"):
            i2c_name = arg
    # Вызываем функцию инициализации
    errCode = lib.RI_SDK_InitSDK(logLevel, errTextC)
    if errCode != 0:
        return errCode, errTextC
    # создаем компонент ШИМ с конкретной моделью как исполняемое устройство,получаем дескриптор сервопривода
    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "pwm".encode(), "pca9685".encode(), device["pwm"],
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC

    # создаем компонент i2c адатера
    # Здесь осуществлен примитивное определение подключенной модели адаптера
    # Сначала пробуем создать i2c адаптер модели ch341 и связать с ним ШИМ
    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "i2c_adapter".encode(), i2c_name.encode(),
                                              device["i2c"], errTextC)
    if errCode != 0:
        return errCode, errTextC

    # связываем i2c адаптер с ШИМ по адресу 0x40
    errCode = lib.RI_SDK_LinkPWMToController(device["pwm"], device["i2c"], c_uint8(0x40), errTextC)
    if errCode != 0:
        return errCode, errTextC

    # создаем компонент светодиода с конкретной моделью (ky016) как исполняемое устройство и получаем дескриптор светодиода
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "led".encode(), "ky016".encode(), device["led"],
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # связываем светодиод с ШИМ,передаем значения трех пинов к которым подключен светодиод
    errCode = lib.RI_SDK_LinkLedToController(device["led"], device["pwm"], 15, 14, 13, errTextC)
    if errCode != 0:
        return errCode, errTextC
    # инициализируем сервоприводы
    errCode, errTextC = initServos()
    if errCode != 0:
        return errCode, errTextC
    return errCode, errTextC


# startPosition - переводит сервопривод в стартовое положение
def startPosition(servo):
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    errCode = 0
    # выполняем поворот сервопривода в заданный угол,передаем дескриптор сервопривода,значение угла
    errCode = lib.RI_SDK_exec_ServoDrive_TurnByPulse(servo["descriptor"], servo["start_position"], errTextC)
    if errCode != 0:
        return errCode, errTextC

    time.sleep(0.5)

    return errCode, errTextC


# startPositionAllServo - переводит все сервоприводы в стартовую позицию
def startPositionAllServo():
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    # выполняем одиночное свечение светодиодом,передаем дескриптор светодиода,3 параметра цвета(RGB), и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(device["led"], 255, 0, 0, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # приводим сервоприводы в стартовое положение
    for i in range(len(servos)):
        errCode, errTextC = startPosition(servos[i])
        if errCode != 0:
            return errCode, errTextC
    # выполняем одиночное свечение светодиодом,передаем дескриптор светодиода,3 параметра цвета(RGB), и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(device["led"], 0, 255, 0, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # небольшая пауза для последовательного движения
    time.sleep(0.5)

    return errCode, errTextC


# destructServos - уничтожает сервоприводы
def destructServos():
    errTextC = c_char_p()  # Текст ошибки. C type: char*

    for i in range(len(servos)):
        errCode = lib.RI_SDK_DestroyComponent(servos[i]["descriptor"], errTextC)
        if errCode != 0:
            return errCode, errTextC

    return errCode, errTextC


# destruct - уничтожает все компоненты и библиотеку
def destruct():
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    # выполняем одиночное свечение светодиодом,передаем дескриптор светодиода,3 параметра цвета(RGB), и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(device["led"], 255, 0, 0, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # уничтожаем сервоприводы
    errCode, errTextC = destructServos()
    if errCode != 0:
        return errCode, errTextC
    # останавливаем свечение светодиода с определенным дескриптором
    errCode = lib.RI_SDK_exec_RGB_LED_Stop(device["led"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    # удаляем компонент светодиода по дескриптору
    errCode = lib.RI_SDK_DestroyComponent(device["led"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    # сбрасываем все порты на ШИМ
    errCode = lib.RI_SDK_sigmod_PWM_ResetAll(device["pwm"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    # удаляем компонент ШИМ
    errCode = lib.RI_SDK_DestroyComponent(device["pwm"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    # удаляем компонент i2c
    errCode = lib.RI_SDK_DestroyComponent(device["i2c"], errTextC)
    if errCode != 0:
        return errCode, errTextC
    # удаляем sdk со всеми компонентами в реестре компонентов
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC

    return errCode, errTextC

# rotateBody - вращение тела в указанный угол
def rotateBody(angle, speed):
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    # выполняем мигание с заданной частотой,передаем дескриптор светодиода,3 параметра цвета(RGB),частоту,продолжительность и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency(device["led"], 0, 255, 0, 5, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор тела,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["body"], angle, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["clawRotate"], 45, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["clawRotate"], -45, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC

    return errCode, errTextC


# get - берет кубик
def get():
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    # выполняем мерцание светодиодом,передаем дескриптор светодиода,3 параметра цвета(RGB),продолжительность,кол-во повторений и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency(device["led"], 0, 0, 255, 500, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowR"], arrowR_over_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowL"], arrowL_over_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["claw"], claw_unclenched_position, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowR"], (arrowR_cube_position - arrowR_over_cube_position),
                                              speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowL"], (arrowL_cube_position - arrowL_over_cube_position),
                                              speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["claw"], (-1) * claw_unclenched_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowR"], (-1) * arrowR_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowL"], (-1) * arrowL_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC

    return errCode, errTextC


# put - кладет кубик
def put():
    errTextC = create_string_buffer(1000)  # Текст ошибки. C type: char*
    # выполняем мерцание светодиодом,передаем дескриптор светодиода,3 параметра цвета(RGB),продолжительность,кол-во повторений и включаем асинхронный режим работы
    errCode = lib.RI_SDK_exec_RGB_LED_Flicker(device["led"], 0, 0, 255, 500, 0, c_bool(True), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowR"], arrowR_cube_position, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowL"], arrowL_cube_position, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["claw"], claw_unclenched_position, speed, c_bool(False), errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowR"], (-1) * arrowR_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор стрелы,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["arrowL"], (-1) * arrowL_cube_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC
    # выполняем поворот на заданный угол,передаем дескриптор клешни,угол,скорость и асинхронный режим работы
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(device["claw"], (-1) * claw_unclenched_position, speed, c_bool(False),
                                              errTextC)
    if errCode != 0:
        return errCode, errTextC

    return errCode, errTextC


# start - запускает демо программу
def start():
    # Инициализируем библиотеку и компоненты
    errCode, errText = initDevice()
    if errCode != 0:
        return errCode, errText

    # Приводим сервоприводы к стартовой позиции
    errCode, errText = startPositionAllServo()
    if errCode != 0:
        return errCode, errText

    # Двигаем тело к местонахождению кубика
    errCode, errText = rotateBody(35, speed)
    if errCode != 0:
        return errCode, errText

    # Берем кубик
    errCode, errText = get()
    if errCode != 0:
        return errCode, errText

    # Двигаем тело к новому местонахождению кубика
    errCode, errText = rotateBody(-77, speed)
    if errCode != 0:
        return errCode, errText

    # Кладем кубик
    errCode, errText = put()
    if errCode != 0:
        return errCode, errText

    # Возвращаем тело в стартовое положение
    errCode, errText = rotateBody(42, speed)
    if errCode != 0:
        return errCode, errText

    # Уничтожаем компоненты и библиотеку
    errCode, errText = destruct()
    if errCode != 0:
        return errCode, errText
    return errCode, errText


# Главная функция запускающая все остальное
def main():
    errCode, errText = start()
    if errCode != 0:
        print(errCode, decodeErr(errText))
        sys.exit(2)

    print("Success", device)


main()
