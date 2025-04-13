import time
import constant
import sys
from ctypes import *

# Подключение библиотеки ri_sdk
lib = cdll.LoadLibrary("./librisdk.dll")
# Далее необходимо указать типы аргументов 
# для функций библиотеки, которые будут использоваться
lib.RI_SDK_InitSDK.argtypes = [c_int, c_char_p]
lib.RI_SDK_CreateModelComponent.argtypes = [c_char_p, c_char_p, c_char_p, POINTER(c_int), c_char_p]
lib.RI_SDK_LinkPWMToController.argtypes = [c_int, c_int, c_uint8, c_char_p]
lib.RI_SDK_LinkLedToController.argtypes = [c_int, c_int, c_int, c_int, c_int, c_char_p]
lib.RI_SDK_LinkServodriveToController.argtypes = [c_int, c_int, c_int, c_char_p]
lib.RI_SDK_exec_RGB_LED_SinglePulse.argtypes = [c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_TurnByPulse.argtypes = [c_int, c_int, c_char_p]
lib.RI_SDK_DestroyComponent.argtypes = [c_int, c_char_p]
lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
lib.RI_SDK_exec_ServoDrive_GetCurrentAngle.argtypes = [c_int, POINTER(c_int), c_char_p]

errTextC = create_string_buffer(1000)   # Перменная, в которую будет записываться текст ошибки.
errCode = c_int() # Переменная, в которую будет записываться код ошибки.

i2c = c_int() # Переменная, в которой будет храниться дескриптор на объект i2c адаптера.
pwm = c_int() # Переменная, в которой будет храниться дескриптор на объект pwm модулятора.
servoBase = c_int() # Переменная, в которой будет храниться дескриптор на объект управления первым сервоприводом.
servoClaw = c_int() # Переменная, в которой будет храниться дескриптор на объект управления вторым сервоприводом.
servoHorizontal = c_int() # Переменная, в которой будет храниться дескриптор на объект управления третьим сервоприводом.
servoVertical = c_int() # Переменная, в которой будет храниться дескриптор на объект управления четвертым сервоприводом.
led = c_int() # Переменная, в которой будет храниться дескриптор на объект управления светодиодом.

# initRISDK() выполняет инциализацию внутренних компонентов RI SDK.
def initRISDK():
    # Вызов функции RI SDK для инициализации.
    errCode = lib.RI_SDK_InitSDK(2, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2) 

# initRobohand() выполняет инициализацию всех компонентов роборуки с помощью RISDK.
# Компоненты создаются внутри бибилиотеки, и она "запоминает" их для
# последующего использования.
def initRobohand():
    # Инициализация компонента i2c адаптера cp2112.
    createRISDKComponent("connector", "i2c_adapter", "cp2112", i2c)
    # Инициализация компонента ШИМ pca9685 и связывание его с i2c адаптером.
    createRISDKComponent("connector", "pwm", "pca9685", pwm)
    linkPWMToController(pwm, i2c, c_uint8(0x40))
    # Инициализация компонента сервопривода mg90s (основание) и связывание его с ШИМ.
    createRISDKComponent("executor", "servodrive", "mg90s", servoBase)
    linkServoToController(servoBase, pwm, 0)
    # Инициализация компонента сервопривода mg90s (клешня) и связывание его с ШИМ.
    createRISDKComponent("executor", "servodrive", "mg90s", servoClaw)
    linkServoToController(servoClaw, pwm, 1)
    # Инициализация компонента сервопривода mg90s (первая стрела) и связывание его с ШИМ.
    createRISDKComponent("executor", "servodrive", "mg90s", servoHorizontal)
    linkServoToController(servoHorizontal, pwm, 2)
    # Инициализация компонента сервопривода mg90s (вторая стрела) и связывание его с ШИМ.
    createRISDKComponent("executor", "servodrive", "mg90s", servoVertical)
    linkServoToController(servoVertical, pwm, 3)
    # Инициализация компонента светодида ky016 и связывание его с ШИМ.
    createRISDKComponent("executor", "led", "ky016", led)
    linkLEDToController(led, pwm, 15, 14, 13)

# completeAndDestroy() выполняет корректное завершение работы с компонентами RI SDK.
# Освобождается память, занятая компонентами, а затем разрушаются внутренние компоненты ядра RI SDK.
def completeAndDestroy():
    print("Завершение работы с RI SDK")
    # Сброс сигналов со всех портов ШИМ.
    errCode = lib.RI_SDK_sigmod_PWM_ResetAll(pwm, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)
    # Освобождение памяти от всех компонентов RI SDK.
    # Обратите внимание, что компоненты удаляются в порядке, обратном порядку из создания.
    # Удаление сервоприводов.
    destroyRISDKComponent(servoBase)
    destroyRISDKComponent(servoClaw)
    destroyRISDKComponent(servoHorizontal)
    destroyRISDKComponent(servoHorizontal)
    # Удаление светодиода.
    destroyRISDKComponent(led)
    # Удаление ШИМ и i2c адаптера.
    destroyRISDKComponent(pwm)
    destroyRISDKComponent(i2c)
    # Удаление компонентов ядра RI SDK.
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# createRISDKComponent() выполняет создание компонента заданных группы, типа устройства и модели.
# По-сути является "функцией-обёрткой" поверх функции RI SDK.
def createRISDKComponent(group, device, model, descriptor):
    # Вызов функции RI SDK для создания компонента.
    errCode = lib.RI_SDK_CreateModelComponent(group.encode(), device.encode(), model.encode(), descriptor, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# createRISDKComponent() выполняет удаление компонента.
# По-сути является "функцией-обёрткой" поверх функции RI SDK.
def destroyRISDKComponent(component):
    # Вызов функции RI SDK для удаления компонента.
    errCode = lib.RI_SDK_DestroyComponent(component, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# resetPosition() выполняет установку стартового положения всех сервоприводов роборуки.
def resetPosition():
    setServoPulse(servoVertical, constant.SERVO_VERTICAL_START_PULSE)
    setServoPulse(servoHorizontal, constant.SERVO_HORIZONTAL_START_PULSE)
    setServoPulse(servoClaw, constant.SERVO_CLAW_START_PULSE)
    setServoPulse(servoBase, constant.SERVO_BASE_START_PULSE)
    
# setСubePosition() выполняет установку роборуки в положение, 
# которое указывает на позицию первоначальной расстановки кубиков.
def setСubePosition():
    setupServoAngle(servoBase, 145)
    setupServoAngle(servoHorizontal, 45)
    setupServoAngle(servoClaw, 0)
    setupServoAngle(servoVertical, 95)
    time.sleep(1)
    setupServoAngle(servoVertical, 80)
    setupServoAngle(servoVertical, 95)
    time.sleep(1)
    setupServoAngle(servoVertical, 80)
    setupServoAngle(servoVertical, 95)
    time.sleep(1)
    setupServoAngle(servoVertical, 80)
    setupServoAngle(servoVertical, 95)

# showCubePosition() указывает пользователю позицию, в которую неоюходимо установить кубики перед началом работы.
def showCubePosition(isRepeat):
    print("Установка роборуки в стартовое положение")
    # Сброс положения роборуки в стартовое положение (перед демонстрацией).
    if isRepeat != True:
        resetPosition()
    ## Ожидание N секунд, чтобы пользователь успел подготовиться.
    print('Установите два кубика на позицию, которую через {0} секунды покажет роборука'.format(constant.TIMEOUT_SHOW_INSTRUCTION))
    time.sleep(constant.TIMEOUT_SHOW_INSTRUCTION)
    # Перевод роборуки в положение, в котором она указывает на место, в которое нужно установить кубики.
    setСubePosition()
    # Сброс положения роборуки в стартовое положение (демонстрация завершена).
    resetPosition()
    # Вывод пользователю инструкции по выбору последующего действия.
    userInstruction = '''Введите {0}, и нажмите enter, если установили кубики.
Или введите {1}, и нажмите enter чтобы роборука ещё раз указала позицию.
Или введите {2}, и нажмите enter, чтобы завершить выполнение программы\n
'''.format(constant.USER_CHOISE_START, constant.USER_CHOISE_REPEAT, constant.USER_CHOISE_EXIT)
    # Чтение потока ввода пользователя.
    userChoise = input(userInstruction)
    # Если пользователь выбрал повторный показ места установки кубиков, то функция showCubePosition() рекурсивно.
    if userChoise == constant.USER_CHOISE_REPEAT:
        # Выполнение функции рекурсивно.
        return showCubePosition(True)
    else:
        # Выход из функции и возврат выбранного пользователем действия.
        return userChoise

# moveServo(descriptor, angle) выполняет поворот сервопривода на заданный угол.
def moveServo(descriptor, angle):
    # В начале движения светодиод загорается красным.
    ledSinglePulse(constant.MAX_COLOR_VAL, constant.MIN_COLOR_VAL, constant.MIN_COLOR_VAL, 0, True)
    # Вызов функции RI SDK для выполнения поворота на заданный угол.
    errCode = lib.RI_SDK_exec_ServoDrive_Turn(descriptor, angle, constant.SERVO_SPEED, c_bool(False), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)
    # При завершении движения светодиод загорается зелёным.
    ledSinglePulse(constant.MIN_COLOR_VAL, constant.MAX_COLOR_VAL, constant.MIN_COLOR_VAL, 0, True)

# setServoPulse(descriptor, pulse) устанавливает заданный импульс на порту целевого сервопривода.
# Это приводит к повороту сервопривода, соответствующему импульсу.
def setServoPulse(descriptor, pulse):
    # В начале движения светодиод загорается красным.
    ledSinglePulse(constant.MAX_COLOR_VAL, constant.MIN_COLOR_VAL, constant.MIN_COLOR_VAL, 0, True)
    # Вызов функции RI SDK для выполнения поворота соответственно заданного импульса.
    errCode = lib.RI_SDK_exec_ServoDrive_TurnByPulse(descriptor, pulse, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)
    # Таймаут необходим, чтобы компонент успел физически завершить поворот.
    # Это обсуловлено особенностями работы сервопривода и RI SDK. 
    time.sleep(constant.TIMEOUT_SETUP_PULSE)
    # При завершении движения светодиод загорается зелёным.
    ledSinglePulse(constant.MIN_COLOR_VAL, constant.MAX_COLOR_VAL, constant.MIN_COLOR_VAL, 0, True)

# getServoCurrentAngle(descriptor) возвращает значение текущего угла сервопривода.
def getServoCurrentAngle(descriptor):
    servoCurrentAngle = c_int()
    # Вызов функции RI SDK для получения текущего угла сервопривода.
    errCode = lib.RI_SDK_exec_ServoDrive_GetCurrentAngle(descriptor, servoCurrentAngle, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)
    # Возвращается целочисленное значение угла.
    return servoCurrentAngle.value

# linkPWMToController(pwm, i2c, addr) выполняет программное связывание ШИМ с i2c адаптером.
# При этом определяется адрес ШИМ на i2c шине.
def linkPWMToController(pwm, i2c, addr):
    # Вызов функции RI SDK для связывания ШИМ с i2c адаптером.
    errCode = lib.RI_SDK_LinkPWMToController(pwm, i2c, addr, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2) 

# linkLEDToController(led, pwm, rport, gport, bport) выполняет программное связывание светодиода с ШИМ.
# При этом определяются номера портов подключения светодиода (RGB).
def linkLEDToController(led, pwm, rport, gport, bport):
    # Вызов функции RI SDK для связывания светодиода с ШИМ.
    errCode = lib.RI_SDK_LinkLedToController(led, pwm, rport, gport, bport, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# linkServoToController(servo, pwm, port) выполняет программное связывание сервопривода с ШИМ.
# При этом определяется номер порта подключения сервопривода.
def linkServoToController(servo, pwm, port):
    # Вызов функции RI SDK для связывания сервопривода с ШИМ.
    errCode = lib.RI_SDK_LinkServodriveToController(servo, pwm, port, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# ledSinglePulse(r, g, b, t, isAsync) выполняет вызов функции свечения светодиодом заданным цветом.
# Параметр isAsync отвечает за асинхронное выполнение функции свечения.
# Асинхронный режим необходим для того, чтобы одновременно с свечением светодиода 
# могли выполнятся другие действия над компонентами. Например, вращение сервопривода.
# По-сути является "функцией-обёрткой" поверх функции RI SDK.
def ledSinglePulse(r, g, b, t, isAsync):
    # Вызов функции RI SDK для выполнения свечения. 
    errCode = lib.RI_SDK_exec_RGB_LED_SinglePulse(led, r, g, b, t, c_bool(isAsync), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        handleError(errCode)
        sys.exit(2)

# setupServoAngle(servo, angle) выполняет установку заданного угла для сервопривода.
def setupServoAngle(servo, angle):
    # Чтение текущего значения угла.
    currentAngle = getServoCurrentAngle(servo)
    # Вычисление угла, накоторый необходимо выполнить поворот в абсолютном значении.
    different = abs(currentAngle - angle)
    # Если задан текущий угол, то выполнять поворот нет необходимости.
    if different == 0:
        return
    # Если целевой угол меньше текущего, то выполняется смещение в меньшую сторону.
    # Иначе в больщую.
    if angle < currentAngle:
        moveServo(servo, -different)
    else:
        moveServo(servo, different)

# holdFirstCube() - функция, которая выполняет захват в клешню первого кубика.
def holdFirstCube():
    print("Беру первый кубик")
    setupServoAngle(servoBase, 145)
    setupServoAngle(servoHorizontal, 75)
    setupServoAngle(servoClaw, 100)
    setupServoAngle(servoVertical, 40)

# moveFirstCube() - функция, которая выполняет перенос в целевую позицию первого кубика.
def moveFirstCube():
    print("Переношу его в целевую точку")
    setupServoAngle(servoBase, 55)
    setupServoAngle(servoHorizontal, 75)
    setupServoAngle(servoVertical, 125)
    setupServoAngle(servoClaw, 200)
    setupServoAngle(servoVertical, 60)

# holdSecondCube() - функция, которая выполняет захват в клешню второго кубика.
def holdSecondCube():
    print("Беру второй кубик")
    setupServoAngle(servoBase, 146)
    setupServoAngle(servoHorizontal, 45)
    setupServoAngle(servoVertical, 95)
    setupServoAngle(servoClaw, 100)
    setupServoAngle(servoVertical, 20)

# moveSecondCube() - функция, которая выполняет перенос в целевую позицию второго кубика.
def moveSecondCube():
    print("Переношу его в целевую точку")
    setupServoAngle(servoBase, 55)
    setupServoAngle(servoHorizontal, 90)
    setupServoAngle(servoVertical, 100)
    setupServoAngle(servoClaw, 200)
    setupServoAngle(servoVertical, 60)

# runWork() выполняет перемещение всех кубиков из изначальной позиции в целевую.
def runWork():
    print("Начанаю выполнять задание...")
    # Захват первого кубика.
    holdFirstCube()
    # Перенос первого кубика в целевую позицию.
    moveFirstCube()
    # Захват второго кубика.
    holdSecondCube()
    # Перенос второго кубика в целевую позицию.
    moveSecondCube()
    print("Задание выполнено!")

# handleError() - функция обработки кодов ошибок RI SDK для вывода русскоязычного описания ошибки.
def handleError(errCode):
    if errCode == 100001:
        print("Неопределенный режим инициализации")
    elif errCode == 100002:
        print("Недопустимый тип компонента")
    elif errCode == 100003:
        print("Параметр длины буфера больше размера буфера")
    elif errCode == 110001:
        print("Реестр компонентов не инициализирован")
    elif errCode == 110002:
        print("Компонент не найден в реестре")
    elif errCode == 110003:
        print("Дескриптор компонентов уже занят")
    elif errCode == 110004:
        print("Ошибка конструктора компонента")
    elif errCode == 110005:
        print("Реестр компонентов не пуст")
    elif errCode == 120006:
        print("Не удалось сохранить компонент уровня модели устройства в реестр")
    elif errCode == 120007:
        print("Не удалось сохранить компонент уровня группы в реестр")
    elif errCode == 120008:
        print("Неизвестный тип устройства")
    elif errCode == 120009:
        print("Неизвестный тип модели")
    elif errCode == 130001:
        print("Ошибка связывания pwm с адаптером")
    elif errCode == 130002:
        print("Указанный компонент не обладает функционалом связывания")
    elif errCode == 130003:
        print("Указанный компонент не поддерживает протокол связывания")
    elif errCode == 130004:
        print("Ошибка связывания компонента с ШИМ")
    elif errCode == 130005:
        print("Ошибка связывания компонента с ШИМ на порту")
    elif errCode == 211001:
        print("Адрес уже занят")
    elif errCode == 211002:
        print("Ошибка создания соединения")
    elif errCode == 211003:
        print("Ошибка закрытия соединения по адресу")
    elif errCode == 211004:
        print("Подключение к устройству по адресу уже существует")
    elif errCode == 211005:
        print("I2C адаптер еще не расширен до конкретной модели")
    elif errCode == 211006:
        print("Ошибка при создании подключения по I2C")
    elif errCode == 211007:
        print("Ошибка записи одного байта")
    elif errCode == 211008:
        print("Ошибка чтения первого байта")
    elif errCode == 211101:
        print("Адрес уже занят")
    elif errCode == 211102:
        print("Ошибка создания подключения")
    elif errCode == 211103:
        print("Подключение к адресу устройства уже существует")
    elif errCode == 211201:
        print("Адрес уже занят")
    elif errCode == 211202:
        print("Ошибка создания подключения")
    elif errCode == 211203:
        print("Подключение к адресу устройства уже существует")
    elif errCode == 212001:
        print("Не поддерживается такое соотношение разрешения и частоты")
    elif errCode == 212002:
        print("Не задан контроллер соединения")
    elif errCode == 212003:
        print("Неверное значение On")
    elif errCode == 212004:
        print("Неверное значение Off")
    elif errCode == 212005:
        print("Порт уже занят")
    elif errCode == 212006:
        print("Нет порта с данным индексом")
    elif errCode == 212007:
        print("Частота не установлена")
    elif errCode == 212008:
        print("Разрешение не установлено")
    elif errCode == 212009:
        print("Количество портов не установлено")
    elif errCode == 212010:
        print("Тактовая частота не установлена")
    elif errCode == 212011:
        print("Не установлено соединение с контроллером")
    elif errCode == 212012:
        print("Соединение с i2c шиной не существует")
    elif errCode == 212013:
        print("Неверное значение значение импульса в доле от рабочего диапазона")
    elif errCode == 212014:
        print("Максимальный импульс устройства не установлен")
    elif errCode == 212015:
        print("Настройка частоты для определенного порта не поддерживается на этом устройстве")
    elif errCode == 212016:
        print("Получение частоты для определенного порта, не поддерживаемого на этом устройстве")
    elif errCode == 212101:
        print("Нет модели для модулятора PWM")
    elif errCode == 221001:
        print("Выход за пределы рабочего диапазона")
    elif errCode == 221002:
        print("Тип полученного направления вращения не определен")
    elif errCode == 221003:
        print("Рабочий диапазон не может быть равен нулю или быть отрицательным")
    elif errCode == 221004:
        print("ШИМ-модулятор не установлен")
    elif errCode == 221005:
        print("Значение максимальной ширины управляющего импульса не установлено")
    elif errCode == 221006:
        print("Максимальная скорость не установлена")
    elif errCode == 221007:
        print("Количество шагов не может быть больше разрешения")
    elif errCode == 221008:
        print("Тип полученной команды не определен")
    elif errCode == 221009:
        print("Скорость вращения больше максимальной")
    elif errCode == 221010:
        print("Скорость вращения меньше или равна нулю")
    elif errCode == 221011:
        print("Выход за пределы рабочего диапазона вращения")
    elif errCode == 221012:
        print("Целевой угол поворота сервопривода меньше нуля")
    elif errCode == 221013:
        print("Максимальное значение ширины управляющего импульса")
    elif errCode == 221101:
        print("Модель для сервопривода не существует")
    elif errCode == 222001:
        print("Неправильное значение цвета")
    elif errCode == 222002:
        print("Не заданы контроллеры для одного из портов подключения светодиода")
    elif errCode == 222003:
        print("Тип полученной команды не определен")
    elif errCode == 222101:
        print("Модель для светодиода не существует")
    else:
        print("Для кода ошибки {0} отсутствует русскоязычное описание".format(errCode))
