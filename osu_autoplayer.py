import cv2
import numpy as np
import time
import socket
import math

import RPi.GPIO as GPIO
from struct import unpack
from PIL import Image
import pyautogui
from multiprocessing import Process
pyautogui.FAILSAFE = False

# Port number to use for TCP connection
PORT = 28800

# Cursor location
cursor_location = (0, 0)

# Shift limit for incremental mouse movement 
SHIFT_LIMIT = (pyautogui.size()[0]//2, pyautogui.size()[1]//2)
TIME_LIMIT = 2

class CircleTapNote:

    def __init__(self, x, y, inner_radius, outer_radius=None):
        self.x = int(x)
        self.y = int(y)
        self.inner_radius = int(inner_radius)
        self.outer_radius = int(outer_radius) if outer_radius is not None else -1
        self.last_outer_radius = self.outer_radius
        self.time_stamp = time.time()
    
    def __eq__(self, other):
        if not isinstance(other, CircleTapNote):
            return False
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, ir: {self.inner_radius}, " +\
            f"or: {self.outer_radius}, last: {self.last_outer_radius}."

    def approach_circle_space(self):
        return self.outer_radius - self.inner_radius

    # Checks for same center as circle from Hough Circles in format [x, y, radius]
    # same center defined if location is within 5 pixels in both directions
    def same_center_detection(self, circle):
        if abs(self.x - int(circle[0])) < 5 and abs(self.y - int(circle[1]) < 5):
            return True
        else:
            return False

    def manual_outer_radius_update(self, decay):
        if self.outer_radius < 0:
            return
        self.outer_radius -= decay


class OsuAutoplayer:

    def __init__(self, image=None):
        # Most recent screenshot received
        self.image = image
        # Display circle detection variable
        self.disp = False

        # set of large circles (position and radius)
        # self.large_circles = 0
        # set of small circles, should all have same radius as each other
        # and same position as corresponding large circles
        # self.small_circles = 0

        # Mouse click state: False if not pressing down, true if pressing down
        self.mouse_click = False

        self.run_autoplayer = False
        self.system_exit = False

        # threshold for line detection
        self.line_thresh = 100

        # Storing circles using CircleTapNote objects
        self.active_circles = []
        self.active_long_notes = {}

        # true if mouse is currently inside a long note, change cli
        self.long_note = False

        # TCP setup
        self.HOST_IP = input("Provide Host IP Address to connect to: ")
        self.active_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.active_socket.connect((self.HOST_IP, PORT))

        # Approximated approach rate for large circle radius size decay (radius/sec)
        self.approach_rate = -1
        self.approach_rate_data = []

        self.update_timestamp = time.time()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(22, GPIO.FALLING, callback=self.GPIO_callback_22, bouncetime=300)
        GPIO.add_event_detect(23, GPIO.FALLING, callback=self.GPIO_callback_23, bouncetime=300)
        GPIO.add_event_detect(27, GPIO.FALLING, callback=self.GPIO_callback_27, bouncetime=300)

    def GPIO_callback_22(self, ch):
        self.disp = not self.disp
        print(f"Circle display set to {self.disp}.")

    def GPIO_callback_23(self, ch):
        self.system_exit = True
        print("Exit asserted.")

    def GPIO_callback_27(self, ch):
        self.run_autoplayer = not self.run_autoplayer
        print(f"Autoplayer run set to {self.run_autoplayer}.")
        if self.run_autoplayer:
            # Fix cursor position
            # Ensure cursor moves back to main monitor
            pyautogui.moveRel(-10000, 0)
            time.sleep(0.01)
            # Need multiple moves because of limitation from barrier on mouse movement
            pyautogui.moveRel(-10000, 0)
            time.sleep(0.01)
            pyautogui.moveRel(-10000, 0)
            time.sleep(0.01)
            # time.sleep(0.5)
            pyautogui.moveTo(pyautogui.size()[0], 0)
            global cursor_location
            cursor_location = (0, 0)
            print(f"Reset cursor to {cursor_location}.")
        else:
            self.active_circles = []
            self.active_long_notes = {}
            self.approach_rate = -1
            self.approach_rate_data = []
            pyautogui.mouseUp()
            self.long_note = False

    #TODO: wait until new image is received, update image, detect circles,
    # send mouse position update to Osu game
    def update(self):
        while not self.system_exit:
            while self.run_autoplayer:
                # wait for image to be sent from Osu game
                self.receive_image()

                # Detect circles from screenshot
                circles, small_circles = self.detect_circles(self.disp)

                # potentially change this to store long notes instead of looking at each photo
                self.active_long_notes = self.detect_long_notes(self.disp)
                
                # based on these long notes, determine if current mouse position is inside of a long note
                for long_note in self.active_long_notes.items():
                    if (not self.in_long_note(long_note)):
                        self.long_note = True
                        # hold down mouse click when in long note 
                        pyautogui.mouseDown()               
                
                # Update game state
                self.update_circles(circles, small_circles)

                if self.disp:
                    image_rep = np.zeros((300, 400, 3), dtype=np.uint8)
                    image_rep = cv2.cvtColor(image_rep, cv2.COLOR_RGB2BGR)
                    cv2.namedWindow("game state")
                    cv2.moveWindow("game state", 0, 500)
                    mouse_x, mouse_y = self.reverse_transform_position(cursor_location[0], cursor_location[1])
                    for active_circle in self.active_circles:
                        cv2.circle(image_rep, (active_circle.x, active_circle.y), active_circle.inner_radius, (255, 0, 0), -1)
                        if active_circle.outer_radius > 0:
                            cv2.circle(image_rep, (active_circle.x, active_circle.y), int(active_circle.outer_radius), (0, 255, 0))
                    cv2.circle(image_rep, (mouse_x, mouse_y), 10, (0, 0, 255), -1)
                    cv2.imshow("game state", image_rep)
                    cv2.waitKey(1)
                # Debug to view perceived game state`
                print("[", end='')
                for active_circle in self.active_circles:
                    print(active_circle, end='; ')
                print("]")
                # print(self.active_circles) 
                self.update_timestamp = time.time()
                # Move mouse as necessary
                self.update_mouse(small_circles)

                # Prioritize system exit over running autoplayer
                if self.system_exit:
                    break
        self.close_connection()

    def close_connection(self):
        self.active_socket.sendall(b'')
        self.active_socket.close()
    
    @staticmethod
    def transform_position(x, y):
        initial_pos = (300, 40)
        scaling_x = (1620 - 300) / 400
        scaling_y = (1080 - 40) / 300
        return (int(initial_pos[0] + scaling_x * x), int(initial_pos[1] + scaling_y * y))

    @staticmethod
    def reverse_transform_position(x, y):
        scaling_x = (1620 - 300) / 400
        scaling_y = (1080 - 40) / 300
        return (int((x - 300) / scaling_x), int((y - 40) / scaling_y))
    
    def move_cursor(self, x, y):
        global cursor_location
        dest_x, dest_y = self.transform_position(x, y)
        relative_x = dest_x - cursor_location[0]
        relative_y = dest_y - cursor_location[1]
        if (dest_x, dest_y) != cursor_location:
            print(f"Moving to ({dest_x}, {dest_y}) with relative ({relative_x}, {relative_y}) for original cooridinates of ({x}, {y}).")
        # Incremental movement needed because of barrier limitations
        movement_x = 0
        movement_y = 0
        while (movement_x != relative_x or movement_y != relative_y):
            if abs(relative_x - movement_x) < SHIFT_LIMIT[0]:
                shift_x = relative_x - movement_x
            elif relative_x - movement_x < 0:
                shift_x = -SHIFT_LIMIT[0]
            else:
                shift_x = SHIFT_LIMIT[0]
            if abs(relative_y - movement_y) < SHIFT_LIMIT[1]:
                shift_y = relative_y - movement_y
            elif relative_y - movement_y < 0:
                shift_y = -SHIFT_LIMIT[1]
            else:
                shift_y = SHIFT_LIMIT[1]
            pyautogui.moveRel(shift_x, shift_y)
            time.sleep(0.01)
            movement_x += shift_x
            movement_y += shift_y
            print(f"Shifted by ({shift_x}, {shift_y}).")
        cursor_location = (dest_x, dest_y)

    def receive_image(self):
        # Send Image request
        self.active_socket.sendall(b'1') 
        
        # Get image length
        packed_length = self.active_socket.recv(8)
        (length, ) = unpack('>Q', packed_length)
        data = b''
        while len(data) < length:
            load_bytes = 4096 if length - len(data) > 4096 else length - len(data)
            data += self.active_socket.recv(load_bytes)
        # Send back ack
        self.active_socket.sendall(b'1')
        self.image = np.array(Image.frombytes('L', (400, 300), data, decoder_name='raw'))

    # TODO: based on current mouse location and location of circles,
    # decide where to move mouse to
    def update_mouse(self, small_circles):
        # if we are in a long note, move cursor to nearest small circle
        if (self.long_note):
            # find circle with center closest to pyautogui.position()
            min_dist = np.inf
            min_pos = 0, 0
            if small_circles is not None:
                for i in small_circles[0,:]:
                    # only consider non-active circles to avoid jumping around
                    if i not in self.active_circles:
                        test_x, test_y = self.transform_position(i[0], i[1])
                        dist = abs(cursor_location[0] - test_x) + abs(cursor_location[1] - test_y)
                        if dist < min_dist:
                            min_dist = dist
                            min_pos = i[0], i[1]
                self.move_cursor(min_pos[0], min_pos[1])

        # if any concentric circles have radius within 10 of each other,
        # move mouse to that location and click
        circle_dists = []
        for known_circle in self.active_circles:
            radius_difference = known_circle.approach_circle_space()
            circle_dists.append(radius_difference if radius_difference >= 0 else 1000)
        if circle_dists:
            while circle_dists != []:
                min_idx = circle_dists.index(min(circle_dists))
                print(min_idx, len(circle_dists), len(self.active_circles))
                next_circle = self.active_circles[min_idx]
                self.move_cursor(next_circle.x, next_circle.y)
                if circle_dists[min_idx] < 10:
                    print("Clicking on last shown position.")
                    # if we reach next circle, no longer in long note
                    if (self.long_note):
                        self.long_note = False
                        pyautogui.mouseUp()
                    # self.long_note = 
                    pyautogui.leftClick()
                    self.active_circles.remove(next_circle)
                    circle_dists.remove(circle_dists[min_idx])
                else:
                    break

    
    # returns true if the given point is inside a long note
    def in_long_note(self, long_note):
        x, y = self.reverse_transform_position(cursor_location[0], cursor_location[1])
        (r1, r2), theta = long_note

        temp = -x*math.cos(theta) + y*math.sin(theta)

        # first check if x,y is below upper bound of long note
        if (temp - r1 < 0):
            return False

        # then check if x,y is above lower bound of long note
        if (temp - r2 < 0):
            return False
        return True
        
    
    # detects large and small circles in current image
    # (TODO: parameters should not be hard coded in future)

    # Can change the minimum distance to different value
    @staticmethod
    def same_center_detection(circle0, circle1):
        if abs(int(circle0[0]) - int(circle1[0])) < 5 and abs(int(circle0[1]) - int(circle1[1])) < 5:
            return True
        else: 
            return False

    def detect_circles(self, disp=False):
        # start = time.time()
        
        # Large circle detection
        minDist = 15
        param1 = 30 #500
        param2 = 100 #200 #smaller value-> more false circles
        minRadius = 25
        maxRadius = 80
        circles = cv2.HoughCircles(self.image, cv2.HOUGH_GRADIENT, 1, minDist,
            param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

        # small circle detection
        minRadius = 10
        maxRadius = 25
        param2 = 60

        small_circles = cv2.HoughCircles(self.image, cv2.HOUGH_GRADIENT, 1, minDist,
            param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
        # print("Took", time.time()-start, "seconds")
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
            cv2.waitKey(1)
        return circles, small_circles
        # print(circles)
        # print(small_circles)
    
    def detect_long_notes(self, disp=False):
        edges = cv2.Canny(self.image,50,250, None)
        lines = cv2.HoughLines(edges,1,np.pi/180,self.line_thresh,None,0,0)
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
                    if (abs(lines[i][0][0] - line2[0]) < 7 and abs(lines[i][0][1] - line2[1]) < .2):
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
                        if (r1 < r2):
                            longNotes[(r1,r2)] = (t1+t2)/2
                        else:
                            longNotes[(r2,r1)] = (t1+t2)/2
                        break
        if (disp):
            cdst = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
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
            cv2.imshow("line detection", cdst)

        return longNotes

    def manual_update_approach_radius(self, index, ):
        # prev_radius = self.active_circles[index].last_outer_radius
        # if prev_radius < self.active_circles[index].outer_radius:
        #     return
        decay = self.approach_rate * (time.time() - self.update_timestamp)
        self.active_circles[index].manual_outer_radius_update(decay)
    #     self.active_circles[index][4] = self.active_circles[index][3]
    #     self.active_circles[index][3] -= prev_radius - self.active_circles[index][3]

    def update_circles(self, circles, small_circles):
        approach_rate_tag = False
        if small_circles is None:
            return
        updated_circles = [False for _ in self.active_circles] # array to track which circles have to be manually updated
        for small_circle in small_circles[0,:]:
            if small_circle[2] < 10:
                continue
            known = False
            for i, known_circle in enumerate(self.active_circles):
                if known_circle.same_center_detection(small_circle):
                    known = True
                    break
            if known and circles is not None: # update outer radius if seen
                known_circle = self.active_circles[i]
                for outer_circle in circles[0, :]:
                    if known_circle.same_center_detection(outer_circle):
                        if known_circle.last_outer_radius > 0 and not approach_rate_tag:
                            approach_rate_tag = True
                            approach_rate_sample = (known_circle.last_outer_radius - int(outer_circle[2])) / (time.time() - self.update_timestamp)
                            self.approach_rate_data.append(approach_rate_sample)
                            self.approach_rate = sum(self.approach_rate_data) / len(self.approach_rate_data)
                        known_circle.outer_radius = int(outer_circle[2])
                        known_circle.last_outer_radius = int(outer_circle[2])                             
                        updated_circles[i] = True
                continue
            elif known:
                continue
            # Else, add new circle to active circles list
            outer_radius = -1
            if circles is not None:
                for outer_circle in circles[0, :]:
                    if self.same_center_detection(small_circle, outer_circle):
                        outer_radius = int(outer_circle[2])
            new_circle = CircleTapNote(small_circle[0], small_circle[1], small_circle[2], outer_radius)
            self.active_circles.append(new_circle)
            updated_circles.append(True)

        if self.approach_rate < 0:
            return

        for i, entry in enumerate(updated_circles):
            if not entry:
                self.manual_update_approach_radius(i)
        for entry in self.active_circles:
            if (time.time() - entry.time_stamp > TIME_LIMIT and entry.last_outer_radius < 0) or (entry.outer_radius < 0 and entry.last_outer_radius > 0):
                self.active_circles.remove(entry)

# img = cv2.imread("images/adj_image_seq_0.bmp", cv2.IMREAD_GRAYSCALE)
# pyautogui.moveTo(pyautogui.size()[0], 0)
autoplayer = OsuAutoplayer()
autoplayer.update()
# autoplayer.detect_circles(True)
