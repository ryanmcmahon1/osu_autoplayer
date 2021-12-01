import io
import socket
from struct import unpack
from PIL import Image
import numpy as np
import time
import statistics

HOST_IP = input("Provide Host IP Address to connect to: ")

PORT = 28800

img_arr = []

# SOCK_STREAM
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST_IP, PORT))
    # Start Transmission
    for i in range(5):
        start_time = time.time()
        s.sendall(b'1') 
        
        # Get image length
        packed_length = s.recv(8)
        (length, ) = unpack('>Q', packed_length)
        data = b''
        while len(data) < length:
            load_bytes = 4096 if length - len(data) > 4096 else length - len(data)
            data += s.recv(load_bytes)
        s.sendall(b'1')
        img = Image.frombytes('L', (400, 300), data, decoder_name='raw')
        img_array = np.array(img)
        print("Image received: ", time.time() - start_time)
        img_arr.append(img)
        #img_arr.append(time.time() - start_time)
    s.sendall(b'')

# SOCK_DGRAM
# with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
#     # Start Transmission
#     s.sendto(b'1', (HOST_IP, PORT))

#     # Get image length
#     packed_length = s.recvfrom(8)
#     (length, ) = unpack('>Q', packed_length)
#     data = b''
#     while len(data) < length:
#       load_bytes = 4096 if length - len(data) > 4096 else length - len(data)
#       data += s.recvfrom(load_bytes)
#     s.sendto(b'1', (HOST_IP, PORT))
#     s.sendto(b'', (HOST_IP, PORT))

for image in img_arr:
    image.show()
#print("Done with 500 iterations: ", statistics.mean(img_arr), max(img_arr)) 


