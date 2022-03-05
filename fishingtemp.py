from pyautogui import *
import pyautogui
from pynput.mouse import Button, Controller
from PIL import Image

mouse = Controller()

time.sleep(2)
pic = pyautogui.screenshot(region=(45,80,1200,500))
width,height = pic.size
t = 1
for x in range(0,width,5):
    for y in range(0,height,5):
        r,g,b,a = pic.getpixel((x,y))

        if ((b in range(80,100))and(r in range(170,200))and(g in range(110,120))):
            print("true")
            t = 2
            mouse.position = (45+x,80+y)
            time.sleep(1)
            mouse.click(Button.left,2)
            time.sleep(0.5)
            break
        else:
            t = 3
    if t==2:
        break

print(t)