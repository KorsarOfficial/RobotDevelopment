rem Скрипт для запуска примера. Пример вызова: .\build.bat

rem Вызываем программу сперва для cp2112
python demo.py -d cp2112
rem Если программа вернула ошибку то вызываем с ch341
if %errorlevel% neq 0 python demo.py -d ch341

