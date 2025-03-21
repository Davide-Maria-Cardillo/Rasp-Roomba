# Rasp-Roomba

Un progetto ROS 2 per controllare un robot Roomba tramite Raspberry Pi.

## Struttura del progetto

- `sensors_node`: legge i sensori del Roomba e pubblica dati su ROS 2.
- `cmd_vel_bridge`: converte i messaggi `/cmd_vel` in comandi seriali per il Roomba.
- `navigation_node`: invia comandi `/cmd_vel` per testare il movimento.
- `launch/roomba_launch.py`: avvia automaticamente tutti i nodi.

## Come avviare

 Compila:
colcon build --symlink-install
source install/setup.bash
Lancia tutto:
ros2 launch roomba_nav roomba_launch.py

 Requisiti:
Raspberry Pi con ROS 2 Humble
Roomba con porta seriale (es. serie 500–900, testato su 865)
Cavo USB-TTL (es. PL2303 o FTDI)
Python 3.10+
Ubuntu 22.04 o simile

 Idee future:
Navigazione autonoma con LiDAR
Mappatura dell’ambiente
Rilevamento ostacoli avanzato
yaml
