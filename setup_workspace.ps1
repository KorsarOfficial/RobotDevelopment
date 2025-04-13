# Скрипт для настройки рабочего пространства ROS2
Write-Host "Настраиваем рабочее пространство ROS2..."

# Создаем директорию рабочего пространства
$workspaceDir = Join-Path $env:USERPROFILE "competitions_ws"
if (-not (Test-Path $workspaceDir)) {
    New-Item -Path $workspaceDir -ItemType Directory -Force
    Write-Host "Создана директория $workspaceDir"
}

# Переходим в директорию рабочего пространства
Set-Location $workspaceDir

# Клонируем репозиторий
if (-not (Test-Path (Join-Path $workspaceDir "src"))) {
    Write-Host "Клонируем репозиторий competitions_webots..."
    git clone https://github.com/ArtemVinokurov/competitions_webots.git src
}

# Устанавливаем зависимости
Write-Host "Устанавливаем зависимости ROS2..."
# Эти команды нужно выполнять в консоли с административными правами
# Вместо этого выведем список команд для ручной установки
@"
Выполните следующие команды в PowerShell с правами администратора:

choco install -y ros-humble-ros2-control
choco install -y ros-humble-ros2-controllers
choco install -y ros-humble-controller-manager
choco install -y ros-humble-webots-ros2
choco install -y ros-humble-moveit
choco install -y ros-humble-joint-state-broadcaster
choco install -y ros-humble-joint-state-publisher
choco install -y python-colcon-common-extensions
"@ | Out-File -FilePath "install_dependencies.txt"

Write-Host "Список команд для установки зависимостей сохранен в файле install_dependencies.txt"

# Информация по сборке проекта
@"
После установки зависимостей выполните следующие команды для сборки проекта:

cd ~/competitions_ws
call C:\dev\ros2_humble\local_setup.bat
colcon build
source install/setup.bat

Для запуска симулятора:
ros2 launch simulation_pkg simulation.launch.py
"@ | Out-File -FilePath "build_instructions.txt"

Write-Host "Инструкции по сборке проекта сохранены в файле build_instructions.txt"
Write-Host "Настройка рабочего пространства завершена!" 