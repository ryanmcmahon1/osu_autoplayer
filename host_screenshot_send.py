import socket
import pyautogui
import numpy as np
from struct import pack

HOST = socket.gethostbyname(socket.gethostname())

print(f"The host IP is {HOST}")

PORT = 28800

def send_screenshot(socket_conn):

    screenshot_image = bytes(pyautogui.screenshot().tobytes())
    # use struct to make sure we have a consistent endianness on the length
    length = pack('>Q', len(screenshot_image))

    # sendall to make sure it blocks if there's back-pressure on the socket
    socket_conn.sendall(length)
    socket_conn.sendall(screenshot_image)

    socket_conn.recv(1)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print("Sending screenshot")
            send_screenshot(conn)
        print("Closing connection")
