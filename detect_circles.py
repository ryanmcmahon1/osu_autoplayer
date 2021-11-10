import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread("images/osu_pic.jpg")
# cv2.imshow("Input image", img)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
minDist = 50
param1 = 30 #500
param2 = 145 #200 #smaller value-> more false circles
minRadius = 60
maxRadius = 1000

# large circle detection
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, minDist,
param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

minRadius = 30
maxRadius = 60
param2 = 120

# small circle detection
small_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
if small_circles is not None:
    small_circles = np.uint16(np.around(small_circles))
    for i in small_circles[0,:]:
        cv2.circle(img, (i[0], i[1]), i[2], (255, 0, 0), 2)

cv2.imshow("circle detection", img)
cv2.waitKey()
