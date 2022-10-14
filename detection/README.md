### Установка

Для работы с детектором людей надо скачать веса и конфиги для Yolov4-tiny. Также скачаем кофигурацию детектора лиц.
Это делается следующим образом:

```shell
# Создаем папку, в которую скачаем все детекторы
mkdir yolov4 && cd yolov4

# Скачиваем Yolov4-tiny конфиг
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg

# Скачиваем Yolov4-tiny весы
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights

# Скачиваем конфиг детектора лиц в виде .xml файла
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml

# Возвращаемся обратно
cd ..
```

Скачиваем файл `inference_target_face.py` в какую-нибудь папку.

```shell
mkdir tests && cd tests
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/detection/inference_target_face.py
```

### Запуск

Запускаем детекцию! Есть пару флагов которые нужно знать для запуска:
- `-s` or `--source` <path_to_video> or <cam_id>
- `-w` or `--weights` <path_to_yolo_weights>
- `-c` or `--config` <path_to_yolo_config>
- `--face_config` <path_to_face_detector_confif>
- `-p` or `--detect_people` - детектировать людей или нет
- `-f` or `--detect_faces` - детектировать лица или нет
- `-t` or `--detect_target` - детектировать цели или нет
- `-d` or `--downscale_by` <float_num> - во сколько раз уменьшить каждый кадр

### Примеры:

**Пример 1**
```shell
# запуск детектора с вебки с id 0 с детекцией людей и лиц, но без детекции целей
python3 inference_target_face.py -s 0 -p -f -w /home/clover/yolov4/yolov4-tiny.weights -c /home/clover/yolov4/yolov4-tiny.cfg --face_config /home/clover/yolov4/haarcascade_frontalface_default.xml
```

**Пример 2**
```shell
# запуск детектора на видео 'video.mp4' с детекцией людей, лиц и целей.
# ещё размер каждого кадра уменьшится в 1.5 раза
python3 inference_target_face.py -s "video.mp4" -p -f -t -d 1.5 -w /home/clover/yolov4/yolov4-tiny.weights -c /home/clover/yolov4/yolov4-tiny.cfg --face_config /home/clover/yolov4/haarcascade_frontalface_default.xml
```
## Запуск на ROS  
для билда нужно поставить зависимости 3 Python  
```bash
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
pip3 install opencv-python
sudo apt-get install libatlas-base-dev
```
