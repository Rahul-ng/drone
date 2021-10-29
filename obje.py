import time
import cv2 as cv
import numpy as np
import math


prototxt_path = "MobileNetSSD_deploy.prototxt.txt"
model_path = "MobileNetSSD_deploy.caffemodel"

CLASSES = [
    "background",
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
]

net = cv.dnn.readNetFromCaffe(prototxt_path, model_path)


def process_frame_MobileNetSSD(next_frame):
    rgb = cv.cvtColor(next_frame, cv.COLOR_BGR2RGB)
    (H, W) = next_frame.shape[:2]

    blob = cv.dnn.blobFromImage(next_frame, size=(300, 300), ddepth=cv.CV_8U)
    net.setInput(blob, scalefactor=1.0 / 127.5, mean=[127.5, 127.5, 127.5])
    detections = net.forward()

    for i in np.arange(0, detections.shape[2]):

        confidence = detections[0, 0, i, 2]

        if confidence > 0.7:

            idx = int(detections[0, 0, i, 1])

            if CLASSES[idx] != "person":
                continue

            box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
            (startX, startY, endX, endY) = box.astype("int")

            cv.rectangle(next_frame, (startX, startY), (endX, endY), (0, 255, 0), 3)

    return next_frame


def VehicheDetection_UsingMobileNetSSD():
    cap = cv.VideoCapture(0)

    fps = 20

    while True:
        ret, next_frame = cap.read()

        if ret == False:
            break

        next_frame = process_frame_MobileNetSSD(next_frame)

        # write frame
        
        cv.imshow("", next_frame)
        key = cv.waitKey(50)

        if key == 27:
            break

    cap.release()
    cv.destroyAllWindows()


VehicheDetection_UsingMobileNetSSD()
