import argparse
import signal
import sys
import threading

import cv2
import numpy as np
from imutils.object_detection import non_max_suppression

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


STRAIGHT = 'STRAIGHT'
LEFT = 'LEFT'
RIGHT = 'RIGHT'

faces = []
people = []
frame = None


def signal_handler(signal, frame):  # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def detect_people():
    global people
    while True:
        if frame is None:
            continue
        (rects, weights) = person_detector.detectMultiScale(frame)  # winStride=(4, 4), padding=(8, 8), scale=1.05)

        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        rects = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        people = rects


def detect_faces():
    global faces
    while True:
        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_detector.detectMultiScale(gray)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--cascade_classifier', default='/home/clover/weights/haarcascade_frontalface_alt.xml')
    parser.add_argument('-n', '--node_name', default='simple_detection_py')
    parser.add_argument('-i', '--img_in', default='/main_camera/image_raw')
    parser.add_argument('-o', '--img_out', default='/detections')
    parser.add_argument('-d', '--direction_topic', default='/direction')

    args = parser.parse_args()
    face_detector_file = args.cascade_classifier
    node_name = args.node_name
    img_in_topic = args.img_in
    img_out_topic = args.img_out
    direction_topic = args.direction_topic

    face_detector = cv2.CascadeClassifier(face_detector_file)

    person_detector = cv2.HOGDescriptor()
    person_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    rospy.init_node(node_name)
    image_pub = rospy.Publisher(img_out_topic, Image, queue_size=1)
    direction_pub = rospy.Publisher(direction_topic, String, queue_size=1)

    face_detection_thread = threading.Thread(target=detect_faces)
    people_detection_thread = threading.Thread(target=detect_people)

    face_detection_thread.start()
    people_detection_thread.start()

    bridge = CvBridge()

    try:
        while True:
            frame = bridge.imgmsg_to_cv2(rospy.wait_for_message(img_in_topic, Image), 'bgr8')
            image_h, image_w, _ = frame.shape

            labeled_frame = frame.copy()

            target_up = 0
            target_down = 0
            for (x1, y1, x2, y2) in people:
                weight = (x2-x1)*(y2-y1)
                xc = (x1+x2)/2
                target_up += xc*weight
                target_down += weight
                cv2.rectangle(labeled_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            for face in faces:
                x, y, w, h = face
                cv2.rectangle(labeled_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            if target_down:
                col = target_up/target_down
                if 0 <= col <= (1 / 3)*image_w:
                    direction = LEFT
                elif (1 / 3) * image_w <= col <= (2 / 3) * image_w:
                    direction = STRAIGHT
                else:
                    direction = RIGHT
            else:
                direction = STRAIGHT

            image_pub.publish(bridge.cv2_to_imgmsg(labeled_frame, 'bgr8'))
            direction_pub.publish(direction)
    except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
        exit(1)
