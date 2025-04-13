# main() - функция-"точка входа" в программах на Python и многих других языках.
def main():
    print("Это пример написанной на Python программы для роборуки")
    print("Инициализация RI SDK")
    # Инициализация RI SDK.
    robohand.initRISDK()
    print("Инициализация роборуки")
    # Инициализация компонентов роборуки.
    robohand.initRobohand()
    # Старт основного "рабочего цикла" роборуки для этого задания.
    workCycle()
    robohand.completeAndDestroy()

# workCycle() - функция "рабочего цикла" роборуки для этого задания.
def workCycle():
    # Запуск функции демонстрации исходной позиции кубиков.
    userChoise = robohand.showCubePosition(False)
    # Если пользователь выберет старт прораммы, то...
    if userChoise == constant.USER_CHOISE_START:
        # ... то выполняется функция переноса кубиков в целевую позицию.
        robohand.runWork()
    # После завершения пользователю предлагается повторить упражнение целиком.
    userInstruction = 'Введите {0}, и нажмите enter, если хотите повторить программу. Чтобы завершить её просто нажмите enter.\n'.format(constant.USER_CHOISE_REPEAT)
    repeatWork = input(userInstruction)
    if repeatWork == constant.USER_CHOISE_REPEAT:
        # Рекурсивный вызов функции "рабочего цикла".3
        return workCycle()
    else:
        # Или выход из функции "рабочего цикла".
        return

# Вызов функции-"точки входа" в программу.
main()