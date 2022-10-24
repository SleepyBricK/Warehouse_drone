from argparse import ArgumentParser
import cv2
import time
from threading import Thread, Lock
from itertools import combinations


CONFIDENCE_THRESHOLD = 0.3
NMS_THRESHOLD = 0.4
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]
DOWNSCALE_FRAME_BY = 1

DETECT_FACES = False
DETECT_PEOPLE = False
DETECT_TARGET = False

SOURCE = 1
# SOURCE = 'video.mp4'


yolo_weights = r'D:\darknet\yolov4-tiny.weights'
yolo_config = r'D:\darknet\cfg\yolov4-tiny.cfg'
face_detector_config = r'yolov4/face.xml'

frame = classes = scores = boxes = detection_takes = None
faces = None

lock = Lock()


def avg(*args):
    return sum(args)/len(args)


def get_frame():
    """
        Function that obtains a frame from cv2.VideoCapture.
        :return cv_image
    """
    ret, frame = cap.read()
    if not ret:
        return None
    else:
        return frame


def detect_people():
    """
        Function that detects peoples in the frame.
        It runs in a thread.
    """
    global classes, scores, boxes, detection_takes
    while True:
        if frame is None:
            continue
        det_start = time.time()
        classes, scores, boxes = model.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
        det_end = time.time()
        detection_takes = round(det_end - det_start, 2)


def detect_faces():
    """
    Function that detects faces in the frame.
    It runs in a thread.
    """
    global faces
    while True:
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect faces
        faces = face_detector.detectMultiScale(gray, 1.1, 4)


def define_target(detections):
    """
    Target detection algorithm that find the closest person or the biggest group.
    :param detections: list of all detected people
    :return: x, y, w, h of the target
    """
    pairs = {}
    pair_indices = combinations(list(range(len(detections))), 2)
    biggest_area = 0
    biggest_area_idx = None

    for i, (*a, area) in enumerate(detections):
        if area > biggest_area:
            biggest_area = area
            biggest_area_idx = i

    for i1, i2 in pair_indices:
        x1, y1, w1, h1, area1 = detections[i1]
        x2, y2, w2, h2, area2 = detections[i2]
        xc1, yc1 = x1 + w1 / 2, y1 + h1 / 2
        xc2, yc2 = x2 + w2 / 2, y2 + h2 / 2
        x_gap = abs(xc2-xc1)-(w1/2+w2/2)
        y_gap = abs(y2-y1)
        if x_gap < max(w1, w2) and y_gap < avg(h1, h2):
            if i1 not in pairs:
                pairs[i1] = {'area_sum': area1, 'pairs': [], 'x1': x1, 'y1': y1, 'x2': x1+w1, 'y2': y1+h1}
            pairs[i1]['pairs'].append(i2)
            pairs[i1]['area_sum'] += area2

            pairs[i1]['x1'] = min(pairs[i1]['x1'], x2)
            pairs[i1]['y1'] = min(pairs[i1]['y1'], y2)
            pairs[i1]['x2'] = max(pairs[i1]['x2'], x2+w2)
            pairs[i1]['y2'] = max(pairs[i1]['y2'], y2+h2)

    if pairs:
        # pairs_items = sorted(pairs.items(), key=lambda x: len(x[1]['pairs']))
        pairs_items = sorted(pairs.items(), key=lambda x: x[1]['area_sum'])
        biggest_group = pairs_items[-1][1]
        x1, y1, x2, y2 = biggest_group['x1'], biggest_group['y1'], biggest_group['x2'], biggest_group['y2']
        w, h = x2-x1, y2-y1
        return x1, y1, w, h
    elif biggest_area_idx is not None:
        x, y, w, h, _ = detections[biggest_area_idx]
        x, y = x-5, y-5
        w, h = w+10, h+10
        return x, y, w, h
    else:
        return 50, 50, 20, 20


# def define_target2(detections, image_width, image_height):
#     img = np.zeros((image_height, image_width))
#     print(img)


def load_model(weights_file, config_file, input_size=(416, 416)):
    """
    Function for loading Yolo model
    :param weights_file: path to .weights file
    :param config_file: path to .cfg file
    :param input_size: size of image model will work with
    :return:
    """
    net = cv2.dnn.readNet(weights_file, config_file)

    model = cv2.dnn_DetectionModel(net)
    model.setInputParams(size=input_size, scale=1 / 255, swapRB=True)

    return model


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-s', '--source', default=str(SOURCE), type=str)
    parser.add_argument('-w', '--weights', default=yolo_weights)
    parser.add_argument('-c', '--config', default=yolo_config)
    parser.add_argument('--face_config', default=face_detector_config)
    parser.add_argument('-p', '--detect_people', action='store_true', default=DETECT_PEOPLE)
    parser.add_argument('-f', '--detect_faces', action='store_true', default=DETECT_FACES)
    parser.add_argument('-t', '--detect_target', action='store_true', default=DETECT_TARGET)
    parser.add_argument('-d', '--downscale_by', default=DOWNSCALE_FRAME_BY, type=float)
    args = parser.parse_args()

    SOURCE = args.source
    if SOURCE.isdecimal():
        SOURCE = int(SOURCE)
    yolo_weights = args.weights
    yolo_config = args.config
    face_detector_config = args.face_config

    DETECT_PEOPLE = args.detect_people
    DETECT_FACES = args.detect_faces
    DETECT_TARGET = args.detect_target

    DOWNSCALE_FRAME_BY = args.downscale_by

    cap = cv2.VideoCapture(SOURCE)

    if DETECT_PEOPLE:
        model = load_model(yolo_weights, yolo_config)
        people_detection_thread = Thread(target=detect_people, daemon=True)
        people_detection_thread.start()

    if DETECT_FACES:
        face_detector = cv2.CascadeClassifier(face_detector_config)
        face_detection_thread = Thread(target=detect_faces, daemon=True)
        face_detection_thread.start()

    cur_target = None
    while cv2.waitKey(1) != 27:
        frame = get_frame()
        time.sleep(0.1)

        start = time.time()
        if frame is None:
            break

        h, w, d = frame.shape
        w, h = int(w/DOWNSCALE_FRAME_BY), int(h/DOWNSCALE_FRAME_BY)
        frame = cv2.resize(frame, (w, h))

        labeled_frame = frame.copy()

        if faces is not None:
            # Draw rectangle around the faces
            for (x, y, w, h) in faces:
                cv2.rectangle(labeled_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        detected = []
        if classes is not None:
            for (classid, score, box) in zip(classes, scores, boxes):
                if classid != 0:
                    continue
                color = COLORS[0]
                label = f"Person [{score*100:.2f}%]"
                x, y, w, h = box
                area = w*h
                detected.append((x, y, w, h, area))
                cv2.rectangle(labeled_frame, (x, y, w, h), color, 2)
                cv2.putText(labeled_frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # TARGET DETECTION AND VISUALISATION
        if DETECT_TARGET:
            target = define_target(detected)
            tx, ty, tw, th = target
            if not cur_target:
                cur_target = [tx, ty, tw, th]

            for i, p in enumerate([tx, ty, tw, th]):
                cur_target[i] += int((p - cur_target[i])*0.7)
            cv2.rectangle(labeled_frame, tuple(cur_target), (0, 0, 255), 3)

        end = time.time()

        fps = round(1 / (end - start), 2) if end - start else 'INF'
        inference_fps = round(1/detection_takes, 2) if detection_takes else 0
        fps_label = f"FPS: {fps} (detection takes: {detection_takes} s; {inference_fps} FPS)"

        cv2.putText(labeled_frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("detections", labeled_frame)


    cap.release()
    cv2.destroyAllWindows()
