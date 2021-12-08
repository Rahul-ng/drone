import cv2
import numpy as np
import control
import time
import imutils

FPS = 25
# control.connect_drone("tcp:127.0.0.1:5762")
# control.configure_PID("PID")
cap = cv2.VideoCapture(0)
OVERRIDE = True
oSpeed = 5
S = 20
tDistance = 5
for_back_velocity = 0
left_right_velocity = 0
up_down_velocity = 0
faceSizes = [1026, 684, 456, 304, 202, 136, 90]
acc = [500, 250, 250, 150, 110, 70, 50]
dimensions = (960, 720)
UDOffset = 150

detector = cv2.CascadeClassifier("haarcascade_frontalface_alt2.xml")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, FPS)

while True:

    #time.sleep(1 / FPS)

    k = cv2.waitKey(20)
    if k == ord("t"):
        print("Taking Off")
        control.arm_and_takeoff(3)
    if k == ord("l"):
        print("Landing")
        control.land()
    if k == 8:
        if not OVERRIDE:
            OVERRIDE = True
            print("OVERRIDE ENABLED")
        else:
            OVERRIDE = False
            print("OVERRIDE DISABLED")
    if k == 27:
        break
    if OVERRIDE:
        # S & W to fly forward & back
        if k == ord("w"):
            for_back_velocity = int(S * oSpeed)
        elif k == ord("s"):
            for_back_velocity = -int(S * oSpeed)
        else:
            for_back_velocity = 0

        # a & d to pan left & right
        if k == ord("d"):
            yaw_velocity = 15
        elif k == ord("a"):
            yaw_velocity = -15
        else:
            yaw_velocity = 0

        # Q & E to fly up & down
        if k == ord("e"):
            up_down_velocity = int(S * oSpeed)
        elif k == ord("q"):
            up_down_velocity = -int(S * oSpeed)
        else:
            up_down_velocity = 0

        # c & z to fly left & right
        if k == ord("c"):
            left_right_velocity = int(S * oSpeed)
        elif k == ord("z"):
            left_right_velocity = -int(S * oSpeed)
        else:
            left_right_velocity = 0
    ok, image = cap.read()
    image = imutils.resize(image, width=920, height=720)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rects = detector.detectMultiScale(
        gray,
        scaleFactor=1.5,
        minNeighbors=2,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE,
    )
    tSize = faceSizes[tDistance]
    cWidth = int(dimensions[0] / 2)
    cHeight = int(dimensions[1] / 2)
    cv2.circle(image, (cWidth, cHeight), 10, (0, 0, 255), 2)
    # loop over the faces and draw a rectangle surrounding each
    for (x, y, w, h) in rects:

        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("Faces", image)

cap.release()
cv2.destroyAllWindows()
