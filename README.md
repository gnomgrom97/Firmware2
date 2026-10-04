# Firmware2
# Firmware2 — симулятор БПЛА (PX4 + Gazebo Garden + ROS 2 Humble)

Виртуальный тренажёр для подготовки к соревнованиям БПЛА.  
Позволяет запускать SITL-симуляцию PX4, управлять дроном через QGroundControl, транслировать видео с камер в ROS 2 и выполнять автономные миссии.

## 📦 Состав

- **Миры Gazebo** (`aerobot_gz_sim/worlds/`) — сцены с QR-метками и стартовой площадкой.
- **Модели** (`aerobot_gz_sim/models/`) — дрон `x500`, камеры (вперёд, вниз, depth), лидар, машинки с QR-кодами.
- **Launch-файлы** (`aerobot_gz_sim/launch/`) — запуск PX4 SITL, мост ROS 2 ↔ Gazebo.
- **Скрипт миссии** (`PX4/mission_px4.py`) — автономный полёт: взлёт → зависание → посадка.
- **Инструкция** (`PX4/PX4 + down_camera.txt`) — пошаговый запуск симулятора и просмотра видео с камеры.

## 🧰 Требования

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Garden
- PX4-Autopilot (SITL)
- QGroundControl 4.4.4
- Python 3 + MAVSDK (`mavsdk-grpc`)
- Пакеты: `ros-humble-ros-gzgarden-bridge`, `ros-humble-rqt-image-view`

## 🚀 Быстрый запуск

См. инструкцию: [`src/aerobot_gz_sim/PX4/PX4 + down_camera.txt`](src/aerobot_gz_sim/PX4/PX4%20%2B%20down_camera.txt)

## 📡 Топики ROS 2

| Топик | Описание |
|-------|----------|
| `/uav1/camera` | камера вперёд |
| `/uav1/camera_down` | камера вниз |
| `/uav1/depth_camera` | depth-камера |
| `/uav1/scan` | лидар |
| `/clock` | время симуляции |

## 📝 Лицензия

Учебный проект.
