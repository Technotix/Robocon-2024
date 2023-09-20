import cv2
import numpy as np
# print("Package Imported")

# images
# img = cv2.imread("Man city.jpeg")
# cv2.imshow("Output", img)
# cv2.waitKey(0)

# videos
# cap = cv2.VideoCapture("test_video.mp4")
# while True:
#     success, img = cap.read()
#     cv2.imshow("Video", img)
#     if cv2.waitKey(1) & 0xFF ==ord('q'):
#         break

# webcam
# cap = cv2.VideoCapture(0)
# cap.set(3, 640)
# cap.set(4, 480)
# cap.set(10,100)
# while True:
#     success, img = cap.read()
#     cv2.imshow("Video", img)
#     if cv2.waitKey(1) & 0xFF ==ord('q'):
#        break

# functions
# img = cv2.imread("Man city.jpeg")
# kernel = np.ones((5,5),np.uint8)
#
# imgGray =cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# imgBlur = cv2.GaussianBlur(imgGray,(7,7),0)
# imgCanny = cv2.Canny(img,150,200)
# imgDilation = cv2.dilate(imgCanny, kernel, iterations = 1)
# imgEroded = cv2.erode(imgDilation, kernel, iterations = 1)
#
# cv2.imshow("Gray Image", imgGray)
# cv2.imshow("Blur Image", imgBlur)
# cv2.imshow("Canny Image", imgCanny)
# cv2.imshow("Dilation Image", imgDilation)
# cv2.imshow("Eroded Image", imgEroded)
# cv2.waitKey(0)

# Resize
# img = cv2.imread("Man city.jpeg")
# print(img)
#
#
# imgResize = cv2.resize(img,(1000,500))
# cv2.imshow("Image",img)
# cv2.imshow("Image Resize",imgResize)
# cv2.waitKey(0)

# crop
# img = cv2.imread("Man city.jpeg")
# imgCropped = img[0:200,200:500]
# cv2.imshow("Image Cropped", imgCropped)
#
# cv2.waitKey(0)

# Shapes
# img = np.zeros((512,512,3),np.uint8)
# #img[200:300,100:300]=255,0,0
# #print(img.shape)
#
# cv2.line(img,(0,0),(img.shape[1],img.shape[0]),(0,255,0),3)
# cv2.rectangle(img,(0,0),(250,350),(0,0,255),2)
# cv2.circle(img,(400,50), 30, (255,255,0),5)
# cv2.putText(img, "OPEN CV", (300,200), cv2.FONT_HERSHEY_SIMPLEX,1,(0,150,0),1)
# cv2.imshow("Image", img)
#
# cv2.waitKey(0)

# Warp perspective
# img = cv2.imread("Man city.jpeg")
# width, height = 250,350
# pts1 = np.float32([[111,219],[287,188],[154, 482],[352,440]])
# pts2 = np.float32([[0,0],[width,0],[0,height],[width, height]])
# matrix = cv2.getPerspectiveTransform(pts1, pts2)
# imgOutput = cv2.warpPerspective(img, matrix,(width, height))
# cv2.imshow("Image", img)
# cv2.imshow("Output", imgOutput)
# cv2.waitKey(0)

# stacking
# img = cv2.imread("Man city.jpeg")
# imgResize = cv2.resize(img,(1000,500))
#
# imgHor = np.hstack((imgResize,imgResize))
# cv2.imshow("Horizontal", imgHor)
# imgVer = np.vstack((imgResize,imgResize))
# cv2.imshow("Vertical", imgVer)
# cv2.waitKey(0)

# shapes detecting

# img = cv2.imread("Man city.jpeg")
# imgContour = img.copy()
#
# def getContours(img):
#     contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     for cnt in contours:
#         area = cv2.contourArea(cnt)
#         #print(area)
#         cv2.drawContours(imgContour,cnt,-1,(255,0,0),3)
#         if area>500:
#             cv2.drawContours(imgContour,cnt,-1,(255,0,0),3)
#             peri = cv2.arcLength(cnt,True)
#             #print(peri)
#             approx = cv2.approxPolyDP(cnt,0.02*peri,True)
#             #print(approx)
#             objCor = len(approx)
#             x, y, w, h = cv2.boundingRect(approx)
#
#             if objCor == 3:
#                 objectType = "Tri"
#             elif objCor == 4:
#                 aspRatio = w/float(h)
#                 if aspRatio>0.95 and aspRatio<1.05:
#                     objectType = "Square"
#
#                 else:
#                     objectType = "Rectangle"
#             elif objCor>4:
#                 objectType = "Circle"
#             else:
#                 objectType = "None"
#             cv2.rectangle(imgContour,(x,y),(x+w,y+h), (0,255,0),2)
#             cv2.putText(imgContour, objectType,(x+(w//2)-10,y+(h//2)-10),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,0),2)
#
# imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
# imgCanny = cv2.Canny(imgBlur, 50,50)
# imgBlank = np.zeros_like(img)
# getContours(imgCanny)
#
# cv2.imshow("Original",img)
# cv2.imshow("Gray", imgGray)
# cv2.imshow("Blur", imgBlur)
# cv2.imshow("Canny", imgCanny)
# cv2.imshow("Blank", imgBlank)
# cv2.imshow("Contour", imgContour)
# cv2.waitKey(0)

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


