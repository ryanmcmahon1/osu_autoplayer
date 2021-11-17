import cv2
import numpy as np
import time

class OsuAutoplayer:
    def __init__(self, image):
        self.image = image
        # set of large circles (position and radius)
        self.large_circles = 0
        # set of small circles, should all have same radius as each other
        # and same position as corresponding large circles
        self.small_circles = 0
        self.mouse_pos = 0
        self.mouse_click = 0

    #TODO: wait until new image is received, update image, detect circles,
    # send mouse position update to Osu game
    def update(self):
        # while(not system_exit):
        # wait for image to be sent from Osu game
        # set self.image
        # call detect_circles
        # call update_mouse
        # send new mouse location/click status to Osu game
        # loop and repeat
        pass

    # TODO: based on current mouse location and location of circles,
    # decide where to move mouse to
    def update_mouse(self):
        # if any concentric circles have radius within 10 of each other,
        # move mouse to that location and click
        pass
    
    # detects large and small circles in current image
    # (TODO: parameters should not be hard coded in future)
    def detect_circles(self, disp=False):
        start = time.time()
        
        # arge cicle detectin
        minDist = 15
        param1 = 30 #500
        param2 = 100 #200 #smaller value-> more false circles
        minRadius = 30
        maxRadius = 100
        circles = cv2.HoughCircles(self.image//10, cv2.HOUGH_GRADIENT, 1, minDist,
            param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

        # small circle detection
        minRadius = 0
        maxRadius = 30
        param2 = 60

        small_circles = cv2.HoughCircles(self.image//10, cv2.HOUGH_GRADIENT, 1, minDist,
            param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        print("Took", time.time()-start, "seconds")
        if (disp):
            img_color = cv2.cvtColor(self.image, cv2.cv2.COLOR_GRAY2BGR)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0,:]:
                    cv2.circle(img_color, (i[0], i[1]), i[2], (0, 255, 0), 2)
            if small_circles is not None:
                small_circles = np.uint16(np.around(small_circles))
                for i in small_circles[0,:]:
                    cv2.circle(img_color, (i[0], i[1]), i[2], (255, 0, 0), 2)
            cv2.imshow("circle detection", img_color)
            cv2.waitKey()
        

img = cv2.imread("images/adj_image_seq_0.bmp", cv2.IMREAD_GRAYSCALE)
autoplayer = OsuAutoplayer(img)
autoplayer.detect_circles(True)
