# Warehouse_drone
## Для установки симулятора необходимо:  
*Устаноить приложение [VMware](https://www.vmware.com/go/getworkstation-win).  
*Уcтановить образ [ubuntu 18.04](https://releases.ubuntu.com/18.04/ubuntu-18.04.6-desktop-amd64.iso).  
*Собрать образ по [видео](https://www.youtube.com/watch?v=-UA9ZOUk5ws&t=1s&ab_channel=%D0%AE%D0%BB%D0%B8%D1%8F%D0%A8%D0%B8%D1%88%D0%BA%D0%B0%D0%BD%D0%BE%D0%B2%D0%B0) инструкции и настроить колтчество опертивной памяти, CPU и 3D grafics.  
*Установить git:  
```bash
 sudo apt update  
 sudo apt install -y --fix-missing git
 ```  
*Установить этот репозиторий:  
```bash
git clone https://github.com/petayyyy/Warehouse_drone.git
```  
*Перейти в директорию с установочным файлом:  
```bash
cd Warehouse_drone
```  
*Запустить установочный файл:
```bash
bash build_simulator.sh
```  
## Запуск симулятора:  
Запустить в первом терминале:  
```bash
roslaunch plan_manage rviz.launch
```  
Во втором терминале:  
```bash
 roslaunch plan_manage kino_replan.launch # Полет до точки
 ```  
 Либо:  
 ```bash
 roslaunch plan_manage topo_replan.launch # Полет по прямой
 ```  
## Исходный проект-алгоритм:  
[Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
