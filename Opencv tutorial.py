import cv2
import numpy as np
# Face detection
faceCascade = cv2.CascadeClassifier('Resources\haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
cap.set(10,100)


while True:
     success, img = cap.read()
     faces = faceCascade.detectMultiScale(img, 1.1, 4)
     for (x, y, w, h) in faces:
         cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
     cv2.imshow("Video", img)
     if cv2.waitKey(1) & 0xFF ==ord('q'):
        break
cv2.waitKey(0)


