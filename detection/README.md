```shell
cd ~

mkdir weights && cd weights

wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg

wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights

cd ~
```

```shell
mkdir detection && cd detection

wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/detection/main.cpp
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/detection/CMakeLists.txt

mkdir build && cd build
cmake ..
make
```

```shell
./detection --weights=/home/clover/weights/yolov4-tiny.weights --config=/home/clover/weights/yolov4-tiny.cfg --topic_in=/cam_pub --topic_out=/detections
```