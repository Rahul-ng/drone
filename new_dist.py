
import cv2
import numpy as np



cap = cv2.VideoCapture(0)



detector = cv2.CascadeClassifier("haarcascade_frontalface_alt2.xml")


while True:
    ok, image = cap.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rects = detector.detectMultiScale(
        gray,
        scaleFactor=1.05,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.CASCADE_SCALE_IMAGE,
    )
    # loop over the faces and draw a rectangle surrounding each
    for (x, y, w, h) in rects:

        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("Faces", image)

    if cv2.waitKey(1) == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
