@echo off
echo Сборка проекта для Windows...
echo Очистка предыдущих build файлов...
rd /s /q build >nul 2>&1
mkdir build >nul 2>&1
echo Копирование файлов в build...
copy *.py build\ >nul 2>&1
copy librisdk.dll build\ >nul 2>&1
echo Сборка завершена!
echo Для запуска используйте: build\main.py 