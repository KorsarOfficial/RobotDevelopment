#!/bin/bash
echo "Сборка проекта для Linux..."
echo "Очистка предыдущих build файлов..."
rm -rf build
mkdir -p build
echo "Копирование файлов в build..."
cp *.py build/
cp librisdk.so build/ 2>/dev/null || echo "Предупреждение: librisdk.so не найден"
echo "Сборка завершена!"
echo "Для запуска используйте: python build/main.py" 