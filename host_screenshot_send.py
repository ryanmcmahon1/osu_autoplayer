import socket
from PIL.Image import NONE
from PIL.ImageGrab import grabclipboard
import pyautogui
import numpy as np
from struct import pack
from PIL import ImageOps
import time

HOST = socket.gethostbyname(socket.gethostname())

print(f"The host IP is {HOST}")

PORT = 28800

ADDRESS = NONE

def send_screenshot(socket_conn):

    screenshot_image = pyautogui.screenshot()
    grayscale_image = ImageOps.grayscale(screenshot_image)
    cropped_image = grayscale_image.crop((300, 40, 1620, 1080))
    resized_image = cropped_image.resize((400, 300))
    screenshot_image = bytes(resized_image.tobytes())
    # use struct to make sure we have a consistent endianness on the length
    length = pack('>Q', len(screenshot_image))

    # sendall to make sure it blocks if there's back-pressure on the socket
    socket_conn.sendall(length)
    socket_conn.sendall(screenshot_image)

    socket_conn.recv(1)

# def send_screenshot_UDP(socket_conn):

#     screenshot_image = pyautogui.screenshot()
#     grayscale_image = ImageOps.grayscale(screenshot_image)
#     resized_image = grayscale_image.resize((720, 480))
#     screenshot_image = bytes(resized_image.tobytes())
#     # use struct to make sure we have a consistent endianness on the length
#     length = pack('>Q', len(screenshot_image))

#     # sendall to make sure it blocks if there's back-pressure on the socket
#     socket_conn.sendto(length, ADDRESS)
#     socket_conn.sendto(screenshot_image, ADDRESS)

#     socket_conn.recvfrom(1)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: #SOCK_STREAM
    s.bind((HOST, PORT))
    s.listen(1) # SOCK_STREAM
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print("Sending screenshot")
            time_start = time.time()
            send_screenshot(conn)
            print("Screenshot Sent: ", time.time() - time_start)
        print("Closing connection")

    # SOCK_DGRAM (UDP)
    # message, ADDRESS = s.recvfrom(1024)
    # print('Connected by', ADDRESS)
    # while True:
    #     data, _ = s.recvfrom(1024)
    #     if not data:
    #         break
    #     print("Sending screenshot")
    #     send_screenshot_UDP(s)
