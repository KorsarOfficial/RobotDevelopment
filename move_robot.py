import sys
from ctypes import *

def main():

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
    lib.RI_SDK_exec_RGB_LED_Stop.argtypes = [c_int, c_char_p]
    lib.RI_SDK_sigmod_PWM_ResetAll.argtypes = [c_int, c_char_p]
    lib.RI_SDK_DestroySDK.argtypes = [c_bool, c_char_p]
    lib.RI_SDK_exec_RGB_LED_FlashingWithFrequency.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool,
                                                            c_char_p]
    lib.RI_SDK_exec_RGB_LED_Flicker.argtypes = [c_int, c_int, c_int, c_int, c_int, c_int, c_bool, c_char_p]
    lib.RI_SDK_exec_ServoDrive_Turn.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
    lib.RI_SDK_exec_ServoDrive_Rotate.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]
    lib.RI_SDK_exec_ServoDrive_MinStepRotate.argtypes = [c_int, c_int, c_int, c_bool, c_char_p]

    
    errTextC = create_string_buffer(1000)   # Перменная, в которую будет записываться текст ошибки
    errCode = c_int() # Переменная, в которую будет записываться код ошибки
    i2c = c_int() # Переменная, в которой будет храниться дескриптор на объект i2c адаптера
    pwm = c_int() # Переменная, в которой будет храниться дескриптор на объект pwm модулятора
    servo_1 = c_int() # Переменная, в которой будет храниться дескриптор на объект управления первым сервоприводом
    servo_2 = c_int() # Переменная, в которой будет храниться дескриптор на объект управления вторым сервоприводом
    servo_3 = c_int() # Переменная, в которой будет храниться дескриптор на объект управления третьим сервоприводом
    servo_4 = c_int() # Переменная, в которой будет храниться дескриптор на объект управления четвертым сервоприводом
    led = c_int() # Переменная, в которой будет храниться дескриптор на объект управления светодиодом

    # В начале необходмио вызвать функцию инициализации библиотеки
    # В качестве парметра необходимо указать уровень логирования
    # (0 - только верхний уровень, 1, 2, 3 - более подробная трассировка)
    # Здесь указан 2 уровень логирования
    errCode = lib.RI_SDK_InitSDK(2, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    # Инициализация i2c адаптера 
    # Для создания i2c адаптера можно использовать функцию RI_SDK_CreateModelComponent 
    # Эта функция создаст компонент устройства конкретной модели 
    # В качестве аргументов необходимо указать группу устройств, название устройства и его модель 
    # В нашем случае - это группа коннекторов(connector), устройство i2c-адаптер(i2c_adapter) модели ch341. 
    # Так же необходимо передать перменную, в которой будет храниться дескриптор этого компонента 
    # Он необходим для того чтобы указывать функциям библиотеки, какой именно компонент должен выполнить команду 

    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "i2c_adapter".encode(), "ch341".encode(),
                                              i2c, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    # Инициализация шим-модулятора
    # Для создания шим-модулятора адаптера можно использовать функцию RI_SDK_CreateModelComponent 
    # Эта функция создаст компонент устройства конкретной модели 
    # В качестве аргументов необходимо указать группу устройств, название устройства и его модель 
    # В нашем случае - это группа коннекторов(connector), устройство широтно-импульсный модулятор модели pca9685 
    # Так же необходимо передать перменную, в которой будет храниться дескриптор этого компонента 
    # Он необходим, для того чтобы указывать функциям библиотеки, какой именно компонент должен выполнить команду 
    errCode = lib.RI_SDK_CreateModelComponent("connector".encode(), "pwm".encode(), "pca9685".encode(), pwm,
                                              errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 


    # Подключение i2c адаптера к шим-модулятору 
    # Для дальейшей работы, необходимо подключить шим-модулятор к i2c-адаптеру 
    # Для этого нужно использовать функцию RI_SDK_LinkPWMToController 
    # В нее в качестве аргументов необходимо передать дескрипторы шим-модулятора и i2c-фдаптера 
    # Также необходимо указать адрес, по которому будет сохдано подключение 
    # Он задается в байтовом формате. В данном случае подключение осуществляется по адресу 0x40  
    
    errCode = lib.RI_SDK_LinkPWMToController(pwm, i2c, c_uint8(0x40), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    # Инициализация 4-х сервоприводов
    # Для создания сервопривода можно использовать функцию RI_SDK_CreateModelComponent 
    # Эта функция создаст компонент устройства конкретной модели 
    # В качестве аргументов необходимо указать группу устройств, название устройства и его модель 
    # В нашем случае - это группа исполнителей(executor), устройство сервопривод(servodrive) модели mg90s(mg90s) 
    # Так же необходимо передать перменную, в которой будет храниться дескриптор этого компонента 
    # Он необходим, для того чтобы указывать функциям библиотеки, какой именно компонент должен выполнить команду 
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(),
                                                  servo_1, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(),
                                                  servo_2, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(),
                                                  servo_3, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "servodrive".encode(), "mg90s".encode(),
                                                  servo_4, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    # Подключение 4-х сервоприводов к шим модулятору
    # Для дальейшей работы, необходимо подключить сервоприводы к шим-модулятору 
    # Для этого нужно использовать функцию RI_SDK_LinkServodriveToController 
    # В нее в качестве аргументов необходимо передать дескрипторы сервопривода и шим-модулятора 
    # Также необходимо указать номер порта, к которому подключен сервопривод 
    # В данном случае подключение осуществляется по портам от 0 до 3  

    errCode = lib.RI_SDK_LinkServodriveToController(servo_1, pwm, 0, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    
    errCode = lib.RI_SDK_LinkServodriveToController(servo_2, pwm, 1, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    
    errCode = lib.RI_SDK_LinkServodriveToController(servo_3, pwm, 2, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    
    errCode = lib.RI_SDK_LinkServodriveToController(servo_4, pwm, 3, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 

    # Инициализация светодиода
    # Для создания светодиода можно использовать функцию RI_SDK_CreateModelComponent 
    # Эта функция создаст компонент устройства конкретной модели 
    # В качестве аргументов необходимо указать группу устройств, название устройства и его модель 
    # В нашем случае - это группа исполнителей(executor), устройство светодиод(led) модели ky016(ky016) 
    # Так же необходимо передать перменную, в которой будет храниться дескриптор этого компонента 
    # Он необходим, для того чтобы указывать функциям библиотеки, какой именно компонент должен выполнить команду 
    errCode = lib.RI_SDK_CreateModelComponent("executor".encode(), "led".encode(), "ky016".encode(), led,
                                              errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2) 
    
    # Подключение светодиода к шим модулятору
    # Для дальейшей работы, необходимо подключить сервоприводы к шим-модулятору 
    # Для этого нужно использовать функцию RI_SDK_LinkLedToController 
    # В нее в качестве аргументов необходимо передать дескрипторы светодиода и шим-модулятора 
    # Также необходимо указать номера портов для каждого из 3-х основных цветов (красный, зеленый, синий), к которым подключен светодиод 
    # В данном случае подключение осуществляется по портам 15(красный), 14(зеленый), 13(синий) 
    errCode = lib.RI_SDK_LinkLedToController(led, pwm, 15, 14, 13, errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)


    # Поворот сервопривода на заданный угол с заданной угловой скоростью 
    # Для поворота сервопривода на заданный угол нужно использовать функцию RI_SDK_exec_ServoDrive_Turn 
    # В качестве аргументов необходмио в нее передать 
    # 1 - дескриптор того сервопривода, который будет соуществлять движение 
    # 2 - угол на который нужно повернуть 
    # (если число положительное, то движение будет происходить на увеличение угла, если отрицательный, то на уменьшение)
    # 3 - скорость вращения (углов в секунду)
    # 4 - флаг асинхронного режима: true - двигаться асинхронно, false - синхронно 

    errCode = lib.RI_SDK_exec_ServoDrive_Turn(servo_1, 90, 100, c_bool(False), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)

    # Удаление библиотеки
    # Для удаления библиотеки можно всопользоваться функцией RI_SDK_DestroySDK 
    # в нее необходимо передать значение флага полной очистки реестра (true - полностью)
    errCode = lib.RI_SDK_DestroySDK(c_bool(True), errTextC)
    if errCode != 0:
        print(errCode, errTextC.raw.decode())
        sys.exit(2)
    
    print("Success")



main()
