import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

CONFIDENCE_THRESHOLD = 0.1  # 0.2
NMS_THRESHOLD = 0.1  # 0.4
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]


net = cv2.dnn.readNet("/home/pi/weights/yolov4-tiny.weights",
                      "/home/pi/weights/yolov4-tiny.cfg")

model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1 / 255, swapRB=False)

rospy.init_node('computer_vision_sample')
image_pub = rospy.Publisher('/cam_pub', Image, queue_size=1)

bridge = CvBridge()


while True:
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

    # frame = cv2.resize(frame, (0, 0), None, 2, 2)
    frame = cv2.resize(frame, (416, 416))

    start = time.time()
    classes, scores, boxes = model.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    end = time.time()

    start_drawing = time.time()
    for (classid, score, box) in zip(classes, scores, boxes):
        print(classid, score, box)
        if classid != 0:
            continue
        color = COLORS[0]
        label = "Person: " + str(score)
        cv2.rectangle(frame, box, color, 2)
        cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    end_drawing = time.time()

    fps_label = "FPS: %.2f (excluding drawing time of %.2fms)" % (
    1 / (end - start), (end_drawing - start_drawing) * 1000)
    cv2.putText(frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

