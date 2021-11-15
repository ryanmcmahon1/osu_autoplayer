import pyautogui
import time
from PIL import ImageOps

time.sleep(5)

img = pyautogui.screenshot()
grayscale_img = ImageOps.grayscale(img)
print(grayscale_img.mode)
# img_bytes = bytes(img.tobytes())

# img_recovered = Image.frombytes('RGB', (1920, 1080), img_bytes, decoder_name='raw')

cropped_img = grayscale_img.crop((300, 40, 1620, 1080))
resized_img = cropped_img.resize((400, 300))
resized_img.show()
resized_img.save("osu_resized.jpg")


# time.sleep(30)
