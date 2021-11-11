import pyautogui
import matplotlib.pyplot as plt
import numpy as np
import time
from PIL import Image
import io

img = pyautogui.screenshot()
print(img.mode)

img_bytes = bytes(img.tobytes())

img_recovered = Image.frombytes('RGB', (1920, 1080), img_bytes, decoder_name='raw')


plt.imshow(img_recovered)
plt.show()

# time.sleep(30)
