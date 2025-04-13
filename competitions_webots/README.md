# Установка ROS2

Инструкции по установке фреймворка ROS2 Humble можно найти по [cсылке](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Для установки рекомендуется десктопная версия ROS2 (ros-humble-desktop).



# Инструкции по установке симулятора Webots

Для установки симулятора Webots выполните в терминале следующие команды:
```
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
sudo apt install webots
```

После окончания установки добавьте в bashrc команду для экспорта переменной окружения:
```
echo 'export WEBOTS_HOME=/usr/local/webots' >> ~/.bashrc
```

# Установка

Установите инструменты git, используя команду:

```
sudo apt install git-all
```
Установите инструмент для сборки colcon:

```
sudo apt install python3-colcon-common-extensions
```
Создайте рабочее пространство для сборки пакетов симулятора:

```
cd ~ && mkdir competitions_ws
```

Перейдите в рабочее пространство и скачайте пакеты из репозитория в папку src:

```
cd ~/competitions_ws && git clone https://github.com/ArtemVinokurov/competitions_webots.git src
```

Установите все необходимые зависимости и пакеты:
```
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-webots-ros2 \
                 ros-humble-moveit \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-state-publisher
```
Перейдите в рабочее пространство и выполните сборку пакетов, используя colcon build:

```
cd ~/competitions_ws
source /opt/ros/humble/setup.bash
colcon build
```

После окончания сборки обновите переменные окружения внутри рабочего пространства (данную команду необходимо выполнять каждый раз при открытии нового терминала):
```
source install/setup.bash
```

# Запуск

Для запуска симулятора выполните команду в терминале
```
ros2 launch simulation_pkg simulation.launch.py
```






