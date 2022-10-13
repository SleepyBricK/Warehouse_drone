### Установка OpenCV C++

Для установки OpenCV необходимо выполнить следующие команды 
(взял [отсюда](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/)):

```shell
sudo apt install -y g++ cmake make git libgtk2.0-dev pkg-config
```

```shell
git clone https://github.com/opencv/opencv.git
```

```shell
mkdir -p build && cd build
```

```shell
cmake ../opencv
make -j4
```

```shell
sudo make install
```

### Установка Yolo

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

Скачиваем файл `inference_threaded.cpp` и `CMakeLists.txt` в какую-нибудь папку.

```shell
mkdir cpp_tests && cd cpp_tests
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/detection_cpp/inference_threaded.cpp
wget https://raw.githubusercontent.com/petayyyy/Warehouse_drone/main/detection_cpp/CMakeLists.txt
```

### Запуск

Перед запуском необходимо скомпилировать программу. Рекомендую
перед этим очищать папку от предыдущих компиляций (если их не было, ошибки всё равно не будет из-за очистки 
несуществующих файлов). Всё это делает следующая команда:

```shell
rm Makefile cmake_install.cmake CMakeCache.txt inference_threaded ; rm -r CMakeFiles ; cmake .; make
```

После успешной компиляции можете запустить программу:

```shell
./inference_threaded
```