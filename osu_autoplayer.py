import cv2
import numpy as np
import time
import socket
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

        # Storing circles as [x, y, inner radius, outer radius, prev. outer radius], -1 for no seen outer radius
        self.active_circles = []

        # TCP setup
        self.HOST_IP = input("Provide Host IP Address to connect to: ")
        self.active_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.active_socket.connect((self.HOST_IP, PORT))

        # Approximated approach rate for large circle radius size decay (radius/sec)
        self.approach_rate = -1

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

    #TODO: wait until new image is received, update image, detect circles,
    # send mouse position update to Osu game
    def update(self):
        while not self.system_exit:
            while self.run_autoplayer:
                # wait for image to be sent from Osu game
                self.receive_image()

                # Detect circles from screenshot
                circles, small_circles = self.detect_circles(self.disp)
                
                # Update game state
                self.update_circles(circles, small_circles)
                print(self.active_circles) # Debug to view perceived game state

                # Move mouse as necessary
                self.update_mouse()

                # Prioritize system exit over running autoplayer
                if self.system_exit:
                    break
        # call detect_circles
        # update known circles
        # call update_mouse
        # send new mouse location/click status to Osu game
        # loop and repeat
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
    
    def move_cursor(self, x, y):
        global cursor_location
        dest_x, dest_y = self.transform_position(x, y)
        relative_x = dest_x - cursor_location[0]
        relative_y = dest_y - cursor_location[1]
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
    def update_mouse(self):
        # if any concentric circles have radius within 10 of each other,
        # move mouse to that location and click
        circle_dists = []
        for known_circle in self.active_circles:
            radius_difference = int(known_circle[3]) - int(known_circle[2])
            circle_dists.append(radius_difference if radius_difference >= 0 else 1000)
        if circle_dists:
            while circle_dists != []:
                min_idx = circle_dists.index(min(circle_dists))
                next_circle = self.active_circles[min_idx]
                self.move_cursor(next_circle[0], next_circle[1])
                if circle_dists[min_idx] < 10:
                    print("Clicking on last shown position.")
                    pyautogui.leftClick()
                    self.active_circles.remove(next_circle)
                    circle_dists.remove(circle_dists[min_idx])
                else:
                    break
        
    
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
        
        # Large cicle detection
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

    def manual_update_approach_radius(self, index):
        prev_radius = self.active_circles[index][4]
        if prev_radius < 0 or prev_radius < self.active_circles[index][3]:
            return
        self.active_circles[index][4] = self.active_circles[index][3]
        self.active_circles[index][3] -= prev_radius - self.active_circles[index][3]

    def update_circles(self, circles, small_circles):
        if small_circles is None:
            return
        updated_circles = [False for _ in self.active_circles] # array to track which circles have to be manually updated
        for small_circle in small_circles[0,:]:
            if small_circle[2] < 10:
                continue
            known = False
            for i, known_circle in enumerate(self.active_circles):
                if self.same_center_detection(small_circle, known_circle):
                    known = True
                    break
            if known and circles is not None: # update outer radius if seen
                for outer_circle in circles[0, :]:
                    if self.same_center_detection(self.active_circles[i], outer_circle):
                        self.active_circles[i][4] = self.active_circles[i][3]
                        self.active_circles[i][3] = int(outer_circle[2])
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
            new_circle = [int(small_circle[0]), int(small_circle[1]), int(small_circle[2]), outer_radius, -1]
            self.active_circles.append(new_circle)

        for i, entry in enumerate(updated_circles):
            if not entry:
                self.manual_update_approach_radius(i)
    

# img = cv2.imread("images/adj_image_seq_0.bmp", cv2.IMREAD_GRAYSCALE)
# pyautogui.moveTo(pyautogui.size()[0], 0)
autoplayer = OsuAutoplayer()
autoplayer.update()
# autoplayer.detect_circles(True)
