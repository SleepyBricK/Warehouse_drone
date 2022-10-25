# Warehouse_drone
## Готовый симулятор:
https://drive.google.com/file/d/1m_P_oT-o55lH2-66dkMkkzMTlMlx9TSq/view?usp=sharing
## Запуск arduino:
```bash
rosrun rosserial_python serial_node.py /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 _baud:=115200
```
## Для установки симулятора необходимо:  
*Устаноить приложение [VMware](https://www.vmware.com/go/getworkstation-win).  
*Уcтановить образ [ubuntu 18.04](https://releases.ubuntu.com/18.04/ubuntu-18.04.6-desktop-amd64.iso).  
*Собрать образ по [видео](https://www.youtube.com/watch?v=-UA9ZOUk5ws&t=1s&ab_channel=%D0%AE%D0%BB%D0%B8%D1%8F%D0%A8%D0%B8%D1%88%D0%BA%D0%B0%D0%BD%D0%BE%D0%B2%D0%B0) инструкции и настроить колтчество опертивной памяти, CPU и 3D grafics.  
*Установить git:  
```bash
 sudo apt update  
 sudo apt install -y --fix-missing git
 ```  
 https://github.com/Iliaaer/Fast-Planner/
 
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
 
[build realsense gazebo](https://www.youtube.com/watch?v=hpUCG6K5muI&ab_channel=RangelAlvarado)  

[buimd map gazebo](https://github.com/mit-acl/swarm_simulator)  

https://github.com/mzahana/px4_fast_planner

## Установка realsense в ros  
```bash
sudo apt install ros-melodic-realsense2-camera ros-melodic-realsense2-camera-dbgsym ros-melodic-realsense2-description 
 ```  
 Далее, чтобы запустить realsense в rviz :  
 ```bash
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
 ```  
 В rviz мы меняем параметры, на такие же, которые стоят в госе, чтобы было вот так:
 
![image](https://user-images.githubusercontent.com/31032527/195205989-6eaafcf6-48f3-4573-876e-b63b682279f7.png)
## Возможные ошибки при сборке  
Если команда catkin_make не найдено, то закройте терминал и откройте его заного. И потоврите действие заново.


Не добавлены зависимости для генерации сообщений  
```bash  
sudo nano ~/catkin_ws/src/Fast-Planner/uav_simulator/Utils/multi_map_server/CMakeLists.txt
```
Заменить строку (163):
```bash  
add_dependencies(multi_map_visualization multi_map_server_messages_cpp)
```
на эту:
```bash  
add_dependencies(multi_map_visualization multi_map_server_messages_cpp multi_map_server_generate_messages_cpp)
```  
____________________  
```bash
sudo -H pip install jinja2
```
## Сборка симулятора:
https://clovervm.ams3.digitaloceanspaces.com/clover-devel_v0.3.ova  

Установите файлы симуляции Realsense:
```bash
cd ~/catkin_ws/src
git clone https://github.com/issaiass/realsense2_description
git clone https://github.com/issaiass/realsense_gazebo_plugin
cd ..
catkin_make
```
Измените файл спавна дрона clover/clover_description/launch/spawn_drone.launch добавив строчки:
```bash
<node pkg="robot_state_publisher" type="robot_state_publisher"  name="robot_state_publisher">
  <param name="publish_frequency" type="double" value="30.0" />
</node>
```
Измените файл гененрации компонентов дрона clover/clover_description/urdf/clover/clover4.xacro добавив строчки:
```bash
<xacro:arg name="d455" default="true"/>
<xacro:include filename="$(find realsense2_description)/urdf/_d435.urdf.xacro"/>

<xacro:if value="$(arg d455)">
  <xacro:sensor_d435 name="camera" topics_ns="camera" parent="base_link" publish_pointcloud="true">
    <origin xyz="0 0 0.2" rpy="0 0 0" />
  </xacro:sensor_d435>
</xacro:if>
```
## !!!!!!!!!!!!!!!!!!!!!!Дрон теряет управление в симуляторе:
Отключите вес у камеры. ждя этого откройте файл realsense2_description/urdf/_d435.gazebo.xacro и измените значение с 1 на 0 на 20 строке:  
```bash
<gravity>0</gravity>
```

## Измените файлов, чтобы менять топики:
Файл Fast-Planner-master\fast_planner\plan_manage\src\traj_server.cpp - ```bash clover_cmd_pub``` публикация в топик значений траекторий полета с rviz`а
```bash
clover_cmd_pub = node.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 50);
```
Файл Fast-Planner-master\fast_planner\plan_manage\launch\kino_replan.launch - ```bash cloud_topic``` - менйем на топик, куда присылает реальный realsense
И там же параметры ```bash max_vel``` и ```bash max_acc``` - максимальная скорость и ускорение, которых может достичь дрон
```bash 
    <arg name="cloud_topic" value="/camera/depth/color/points"/>
    
    <arg name="max_vel" value="1.0" />
    <arg name="max_acc" value="0.5" />
```
