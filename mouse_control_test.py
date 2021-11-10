import RPi.GPIO as GPIO
import pyautogui
import time
pyautogui.FAILSAFE = False

GPIO.setmode(GPIO.BCM)   # Set for broadcom numbering not board numbers...
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

exit_program = False

def GPIO_callback_27(ch):
    global exit_program
    exit_program = True

GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO_callback_27, bouncetime=300)


#print(pyautogui.size())
#start_time = time.time()
#while (time.time() - start_time < 20):
#	print(pyautogui.position())

pyautogui.moveTo(pyautogui.size()[0], 0) # Hit the edge of the screen to move screens
pyautogui.moveRel(1, 0)
time.sleep(2)

pyautogui.moveRel(250, 250) # Need to use relative movements to control cursor on desktop screen
time.sleep(2)

pyautogui.moveRel(500, 0)
time.sleep(2)

pyautogui.moveRel(500, 500)
#pyautogui.click() #Support for clicks

#start_time = time.time()
#while (time.time() - start_time < 20 and not exit_program):
#	pyautogui.click()
#	time.sleep(0.01)
#	print(exit_program)
time.sleep(2)
