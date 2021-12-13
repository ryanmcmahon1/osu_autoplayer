import cv2
import numpy as np
import time
import socket

import RPi.GPIO as GPIO
from struct import unpack
from PIL import Image
import pyautogui
from multiprocessing import Process, Value, Array
pyautogui.FAILSAFE = False
pyautogui.PAUSE = 0

# Port number to use for TCP connection
PORT = 28800
HOST_IP = input("Provide Host IP Address to connect to: ")
THRESHOLD_DIFF = 1
TIME_LIMIT = 2

# Cursor location
cursor_location = (0, 0)

# Tag used to track image updates
image_tag_w = Value('i', -1, lock=False)

system_exit = Value('i', 0, lock=False)

frame_time = Value('d', 0, lock=False)

delay_adjust = Value('d', 0, lock=False)

# Updated images from main device, to be communicated between parallel processes.
shared_screenshot = Array("i", 400*300, lock=False)

def GPIO_callback_23(ch):
    global system_exit
    system_exit.value = 1
    print("Exit asserted.")

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(23, GPIO.FALLING, callback=GPIO_callback_23, bouncetime=300)


# Shift limit for incremental mouse movement 
SHIFT_LIMIT = (pyautogui.size()[0]//2, pyautogui.size()[1]//2)


class CircleTapNote:

    def __init__(self, x, y, inner_radius, outer_radius=None):
        self.x = int(x)
        self.y = int(y)
        self.inner_radius = int(inner_radius)
        self.outer_radius = int(outer_radius) if outer_radius is not None else -1
        self.last_outer_radius = self.outer_radius
        self.frame_time_accm = 0
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

class ScreenshotReceiver:

    def __init__(self):
        # TCP setup
        self.HOST_IP = HOST_IP
        self.active_socket =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.active_socket.connect((self.HOST_IP, PORT))
        self.frame_timestamp = 0
        self.frame_init_time = 0
    
    def receive_image(self):
        # Send Image request
        self.active_socket.sendall(b'1') 
        
        self.frame_init_time = time.time()

        # Get image length
        packed_length = self.active_socket.recv(8)
        (length, ) = unpack('>Q', packed_length)
        data = b''
        while len(data) < length:
            load_bytes = 4096 if length - len(data) > 4096 else length - len(data)
            data += self.active_socket.recv(load_bytes)
        # Send back ack
        self.active_socket.sendall(b'1')
        image = np.array(Image.frombytes('L', (400, 300), data, decoder_name='raw'))
        self.update_shared_arr(image)
        
    def update_shared_arr(self, image):
        global shared_screenshot, image_tag_w, frame_time, delay_adjust
        image = image.reshape(400*300)
        shared_screenshot[:]= image[:]
        frame_time.value = time.time()
        delay_adjust.value = self.frame_init_time
        image_tag_w.value += 1
    
    def close_connection(self):
        self.active_socket.sendall(b'')
        self.active_socket.close()
    
    def run_screenshot_receive(self):
        global system_exit
        frame_time.value = time.time()
        while not system_exit.value:
            self.receive_image()
        self.close_connection

class OsuAutoplayer:

    def __init__(self, image=None):
        # Most recent screenshot received
        self.image = image
        # Display circle detection variable
        self.disp = False

        self.image_tag_r = -1
        # set of large circles (position and radius)
        # self.large_circles = 0
        # set of small circles, should all have same radius as each other
        # and same position as corresponding large circles
        # self.small_circles = 0

        # Mouse click state: False if not pressing down, true if pressing down
        self.mouse_click = False

        self.run_autoplayer = False

        # Storing circles using CircleTapNote objects
        self.active_circles = []

        # Approximated approach rate for large circle radius size decay (radius/sec)
        self.approach_rate = -1
        self.ar_alpha = 0.3
        self.ar_init = False

        self.update_timestamp = time.time()
        self.frame_timestamp = time.time()
        self.frame_init = False
        self.frame_time = 0


        GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(22, GPIO.FALLING, callback=self.GPIO_callback_22, bouncetime=300)
        GPIO.add_event_detect(27, GPIO.FALLING, callback=self.GPIO_callback_27, bouncetime=300)

    def GPIO_callback_22(self, ch):
        self.disp = not self.disp
        print(f"Circle display set to {self.disp}.")

    def GPIO_callback_27(self, ch):
        self.run_autoplayer = not self.run_autoplayer
        print(f"Autoplayer run set to {self.run_autoplayer}.")
        if self.run_autoplayer:
            # Fix cursor position
            # Ensure cursor moves back to main monitor
            pyautogui.moveRel(-10000, 0)
            time.sleep(0.01)
            # Need multiple moves because of limitation from barrier on mouse movement
            pyautogui.moveRel(-1000, 0)
            time.sleep(0.05)
            pyautogui.moveRel(-1000, 0)
            time.sleep(0.05)
            pyautogui.moveRel(-1000, 0)
            time.sleep(0.05)
            pyautogui.moveRel(-1000, 0)
            time.sleep(0.05)
            pyautogui.moveRel(-1000, 0)
            time.sleep(0.05)
            # time.sleep(0.5)
            pyautogui.moveTo(pyautogui.size()[0], 0)
            global cursor_location
            cursor_location = (0, 0)
            print(f"Reset cursor to {cursor_location}.")
        else:
            self.active_circles = []
            self.approach_rate = -1
            self.approach_rate_data = []

    #TODO: wait until new image is received, update image, detect circles,
    # send mouse position update to Osu game
    def update(self):
        iter_ct = 0
        global system_exit, frame_time, delay_adjust
        while not system_exit.value:
            while self.run_autoplayer:
                time_stamp = time.time()
                circles = None
                small_circles = None
                frame_update = False
                # wait for image to be sent from Osu game
                tag_read = image_tag_w.value
                screenshot_frame_time = frame_time.value
                delay_adj = delay_adjust.value
                if tag_read > 0 and tag_read != self.image_tag_r:
                    
                    self.image = np.array(shared_screenshot[:], dtype=int).reshape((300, 400))
                    self.image = self.image.astype("uint8").copy()
                    self.image_tag_r = tag_read
                    # Detect circles from screenshot
                    circles, small_circles = self.detect_circles(self.disp)
                    if self.frame_init:
                        self.frame_time = screenshot_frame_time - self.frame_timestamp
                    else:
                        self.frame_init = True
                    self.frame_timestamp = screenshot_frame_time
                    frame_update = True
                    print(f"AR: {self.approach_rate}; frametime: {self.frame_time}\n")
                    # Debug to view perceived game state`
                    print("[", end='')
                    for active_circle in self.active_circles:
                        print(active_circle, end='; \n')
                    print("] \n")
                else:
                    time.sleep(0.001)
                post_image_time = time.time()
                # Update game state
                self.update_circles(circles, small_circles, frame_update, delay_adj)
                circle_update = time.time()
                # Debug to view perceived game state`
                # print("[", end='')
                # for active_circle in self.active_circles:
                #     print(active_circle, end='; ')
                # print("]")
                # print(self.active_circles) 
                self.update_timestamp = time.time()
                # Move mouse as necessary
                self.update_mouse()
                mouse_update = time.time()
                loop_time = time.time() - time_stamp
                if loop_time > .2:
                    print(f"Long loop time: {loop_time}; iter {iter_ct}")
                    print(f"image: {post_image_time - time_stamp}, circle: {circle_update - post_image_time}, mouse: {mouse_update - circle_update}")
                
                # Prioritize system exit over running autoplayer
                if system_exit.value:
                    break
                iter_ct += 1

                if iter_ct % 50 == 0:
                    if self.disp:
                        image_rep = np.zeros((300, 400, 3), dtype=np.uint8)
                        image_rep = cv2.cvtColor(image_rep, cv2.COLOR_RGB2BGR)
                        cv2.namedWindow("game state")
                        cv2.moveWindow("game state", 0, 500)
                        for active_circle in self.active_circles:
                            cv2.circle(image_rep, (active_circle.x, active_circle.y), active_circle.inner_radius, (255, 0, 0), -1)
                            if active_circle.outer_radius > 0:
                                cv2.circle(image_rep, (active_circle.x, active_circle.y), int(active_circle.outer_radius), (0, 255, 0))
                        cv2.imshow("game state", image_rep)
                        cv2.waitKey(1)
                    
    
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
        if cursor_location != (dest_x, dest_y):
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
            if self.disp:
                print(f"Shifted by ({shift_x}, {shift_y}).")
        cursor_location = (dest_x, dest_y)

    # TODO: based on current mouse location and location of circles,
    # decide where to move mouse to
    def update_mouse(self):
        # if any concentric circles have radius within 10 of each other,
        # move mouse to that location and click
        circle_dists = []
        for known_circle in self.active_circles:
            radius_difference = known_circle.approach_circle_space()
            circle_dists.append(1000 if (radius_difference < 0 and known_circle.last_outer_radius < 0) else radius_difference)
        if circle_dists:
            while circle_dists != []:
                min_idx = circle_dists.index(min(circle_dists))
                if circle_dists[min_idx] < 20:
                    print(circle_dists)
                if circle_dists[min_idx] != 1000:
                    next_circle = self.active_circles[min_idx]
                    self.move_cursor(next_circle.x, next_circle.y)
                    if circle_dists[min_idx] < THRESHOLD_DIFF:
                        print("Clicking on last shown position.")
                        pyautogui.leftClick()
                        self.active_circles.remove(next_circle)
                        circle_dists.remove(circle_dists[min_idx])
                    else:
                        break
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

    def manual_update_approach_radius(self, index, ):
        prev_radius = self.active_circles[index].last_outer_radius
        if prev_radius < 0 or prev_radius < self.active_circles[index].outer_radius:
            return
        decay = self.approach_rate * (time.time() - self.update_timestamp)
        self.active_circles[index].manual_outer_radius_update(decay)

    def update_circles(self, circles, small_circles, frame_update, delay):
        updated_circles = [False for _ in self.active_circles] # array to track which circles have to be manually updated
        if small_circles is not None:
            for small_circle in small_circles[0,:]:
                if small_circle[2] < 10:
                    continue
                known = False
                i = None
                for i, known_circle in enumerate(self.active_circles):
                    if known_circle.same_center_detection(small_circle):
                        known = True
                        break
                if i is not None and updated_circles[i]:
                    continue
                if known and circles is not None: # update outer radius if seen
                    known_circle = self.active_circles[i]
                    for outer_circle in circles[0, :]:
                        if known_circle.same_center_detection(outer_circle):
                            if known_circle.last_outer_radius > 0:
                                approach_rate_sample = (known_circle.last_outer_radius - int(outer_circle[2])) / (self.frame_time + known_circle.frame_time_accm)
                                if self.ar_init:
                                    self.approach_rate = self.ar_alpha * approach_rate_sample + (1 - self.ar_alpha) * self.approach_rate
                                else:
                                    self.approach_rate = approach_rate_sample
                                    self.ar_init = True
                                print(f"AR sample: {approach_rate_sample}; {known_circle.last_outer_radius}, {int(outer_circle[2])}, {known_circle.frame_time_accm}")
                            if self.approach_rate > 0:
                                adjusted_radius = int(outer_circle[2]) - self.approach_rate * (time.time() - delay)
                            else:
                                adjusted_radius = int(outer_circle[2])
                            known_circle.outer_radius = adjusted_radius
                            known_circle.last_outer_radius = int(outer_circle[2])
                            known_circle.frame_time_accm = 0
                            known_circle.time_stamp = time.time()
                            updated_circles[i] = True
                    continue
                elif known:
                    continue
                # Else, add new circle to active circles list
                print("spawn new circle")
                outer_radius = -1
                if circles is not None:
                    for outer_circle in circles[0, :]:
                        if self.same_center_detection(small_circle, outer_circle):
                            outer_radius = int(outer_circle[2])
                new_circle = CircleTapNote(small_circle[0], small_circle[1], small_circle[2], outer_radius)
                self.active_circles.append(new_circle)
                updated_circles.append(True)

        for i, entry in enumerate(updated_circles):
            if not entry:
                if frame_update:
                    self.active_circles[i].frame_time_accm += self.frame_time
                if self.approach_rate > 0:
                    self.manual_update_approach_radius(i)


        for entry in self.active_circles:
            if time.time() - entry.time_stamp > TIME_LIMIT and entry.last_outer_radius < 0:
                self.active_circles.remove(entry)
        
        
    

def parallel_process():
    screenshot_receiver = ScreenshotReceiver()
    screenshot_receiver.run_screenshot_receive()

# img = cv2.imread("images/adj_image_seq_0.bmp", cv2.IMREAD_GRAYSCALE)
# pyautogui.moveTo(pyautogui.size()[0], 0)
autoplayer = OsuAutoplayer()
parallel_process = Process(target=parallel_process)
parallel_process.start()
autoplayer.update()
parallel_process.join()
# autoplayer.detect_circles(True)
