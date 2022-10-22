### Установка


```shell
# Создаем папку, в которую скачаем все детекторы
mkdir weights && cd weghts

# Скачиваем Yolov4-tiny конфиг
wget https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg

# Скачиваем Yolov4-tiny весы
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights

# Скачиваем конфиг детектора лиц в виде .xml файла
wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_alt.xml

# Возвращаемся обратно
cd ..
```

Скачиваем файл `detector.py` и `detector_threaded.py` в какую-нибудь папку.

```shell
mkdir tests && cd tests
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/simple_detection/detector.py
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/simple_detection/detector_threaded.py
```

### Запуск

Запускаем детекцию! Есть пару флагов которые нужно знать для запуска:
- `-s`, `--source` <path_to_video> or <cam_id>
колько раз уменьшить каждый кадр
- `-c`, `--cascade_classifier`, путь до детектора лиц. default='/home/clover/weights/haarcascade_frontalface_alt.xml'
- `-n`, `--node_name` имя нода default='simple_detection_py'
- `-i`, `--img_in` имя топика, где брать изображения default='/main_camera/image_raw'
- `-o`, `--img_out` имя топика, куда кидать изображение default='/detections'
- `-d`, `--direction_topic` имя топика, куда кидать направление полёта default='/direction'
