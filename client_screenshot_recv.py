import io
import socket
from struct import unpack
from PIL import Image
import numpy as np

HOST_IP = input("Provide Host IP Address to connect to: ")

PORT = 28800

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST_IP, PORT))

    # Start Transmission
    s.sendall(b'1') 
    
    # Get image length
    packed_length = s.recv(8)
    (length, ) = unpack('>Q', packed_length)
    data = b''
    while len(data) < length:
      load_bytes = 4096 if length - len(data) > 4096 else length - len(data)
      data += s.recv(load_bytes)
    s.sendall(b'1')
    s.sendall(b'')

img = Image.frombytes('RGB', (1920, 1080), data, decoder_name='raw')
img_array = np.array(img)

img.show()
print("Image received")
