# Ros Slam Robot

Raspberry Pi 4, Arduino UNO ve RPLIDAR A1 kullanan iki tekerlekli bir ROS 2 Humble eğitim robotu. Bu depo; diferansiyel sürüş hesabı, seri motor köprüsü, robot modeli, SLAM ve Nav2 başlatma dosyalarını tek bir özgün ROS 2 paketi içinde sunar.

> Kod ve dokümantasyon bu depo için sıfırdan hazırlanmıştır. Donanımı çalıştırmadan önce motor yönlerini, teker ölçülerini ve seri port ayarlarını kendi robotuna göre doğrula.

## Fotoğraflar

| Robot | RViz ve lidar |
|---|---|
| ![Robot prototipi](media/robot.jpg) | ![RViz LaserScan](media/rviz-laserscan.png) |

| Haritalama | Navigasyon |
|---|---|
| ![SLAM deneyi](media/slam-mapping.png) | ![Nav2 deneyi](media/nav2-navigation.png) |

## Özellikler

- `/cmd_vel` komutlarını sol ve sağ teker hızlarına dönüştürür.
- Arduino ile satır tabanlı, okunabilir bir seri protokol kullanır.
- Teker enkoderlerinden `/odom` ve `odom -> base_link` dönüşümü üretir.
- Xacro tabanlı iki tekerlekli robot modeli içerir.
- `slam_toolbox`, Nav2 ve RPLIDAR için hazır başlatma dosyaları sağlar.
- Robot ölçüleri, portlar ve hız sınırları YAML üzerinden ayarlanabilir.

## Gereksinimler

- Ubuntu 22.04 ve ROS 2 Humble
- Raspberry Pi 4
- Arduino UNO uyumlu kart
- İki DC motor ve enkoder
- Motor sürücü
- RPLIDAR A1

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rplidar-ros \
  ros-humble-xacro \
  python3-serial \
  python3-colcon-common-extensions
```

## Kurulum

```bash
mkdir -p ~/ros_slam_ws/src
cd ~/ros_slam_ws/src
git clone https://github.com/Furkangnn/Ros_Slam_Robot.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Arduino yazılımını `firmware/ros_slam_motor_controller/ros_slam_motor_controller.ino` dosyasından karta yükle. Ardından `udev/99-ros-slam-robot.rules` içindeki kimlikleri cihazlarına göre düzenle.

## Kullanım

Robot ve lidar:

```bash
ros2 launch ros_slam_robot robot.launch.py
```

Haritalama:

```bash
ros2 launch ros_slam_robot slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/robot_map
```

Kayıtlı harita ile navigasyon:

```bash
ros2 launch ros_slam_robot navigation.launch.py map:=$HOME/robot_map.yaml
```

## Seri protokol

Raspberry Pi, Arduino'ya saniyede 20 kez `V <sol_mps> <sag_mps>` gönderir. Arduino `E <sol_tick> <sag_tick>` yanıtı verir. Bağlantı zaman aşımına uğrarsa iki taraf da motorları durdurur.

## Test

```bash
cd ~/ros_slam_ws
colcon test --packages-select ros_slam_robot
colcon test-result --verbose
```

## Lisans

Bu projenin özgün kodu MIT Lisansı ile yayımlanır. ROS 2 paketleri ve donanım sürücüleri kendi lisanslarına tabidir.
