import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import math

for i in range(10):
    start = time.time()
    filename = "slider_seq_"+str(i)+".bmp"
    filename = "test_images/"+filename
    # img = cv2.imread(filename,0)
    img = cv2.imread(filename)

    # straight line detection
    edges = cv2.Canny(img,50,250, None)
    cdst = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    lines = cv2.HoughLines(edges,1,np.pi/180,150,None,0,0)
    
    # r = (r1, r2) is key, theta is value. r gives the upper and lower lines of rectangle, 
    # theta gives orientation
    lines_2 = []
    longNotes = {}

    if lines is not None:
        # removing lines detected within 5 pixels of each other
        lines_2.append((lines[0][0][0], lines[0][0][1]))
        for i in range(1,len(lines)):
            # print("checking", i)
            duplicate = False
            for line2 in lines_2:
                if (abs(lines[i][0][0] - line2[0]) < 5 and abs(lines[i][0][1] - line2[1]) < .02):
                    # print(lines[i][0][0], line2[0], "are not at least 5 pixels apart and have similar orientation")
                    duplicate = True
            if (not duplicate):
                lines_2.append((lines[i][0][0], lines[i][0][1]))

        # print(lines_2)
        for i in range(len(lines_2)):
            for j in range(i+1, len(lines_2)):
                r1, t1 = (lines_2[i][0], lines_2[i][1])
                r2, t2 = (lines_2[j][0], lines_2[j][1])
                dist = abs(r1 - r2)
                if (dist - 44 < 10 and dist - 44 > -10 and abs(t1 - t2) < .02):
                    longNotes[(r1,r2)] = (t1+t2)/2
                    break

        # print(longNotes)

            # longNotes[(lines[i][0][0], lines[i][0][1])] = 
    colors = [(0,0,255), (0,255,0), (255,0,0)]
    color = 0
    print(longNotes)
    for r, theta in longNotes.items():
        a = math.cos(theta)
        b = math.sin(theta)
        for i in r:
            x0 = a * i
            y0 = b * i
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, colors[color], 3, cv2.LINE_AA)
        color += 1
        if (color > 2): color = 0

    print("Took", time.time()-start, "seconds")
    cv2.imshow(filename, cdst)
    # cv2.imshow(filename, img)
    # cv2.imshow(filename+" mask", mask)
cv2.waitKey()
