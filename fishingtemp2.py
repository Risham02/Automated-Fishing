from ppadb.client import Client
import time
from PIL import Image
from mss import mss
import numpy

# 1080x2400 phone

adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()
if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]

def fish():
    device.shell('input tap 2248 844')
    while True:
        image = device.screencap()

        with open('screen.png','wb') as f:
            f.write(image)

        image = Image.open('screen.png')
        image = numpy.array(image, dtype=numpy.uint8)

        r1,g1,b1,a1 = image[916][1646]
        r2,g2,b2,a2 = image[500][2150]

        if r2 != 163:
            device.shell('input tap 2150 500')
            time.sleep(0.1)
        if r1 != 206:
            return
        print(r,g,b)


t=1
for x in range(300,2040,200):
    for y in range(400,1080,200):

        a = 'input tap'+str(x)+' '+str(y)
        device.shell(a)
        time.sleep(0.1)
        image = device.screencap()
        with open('screen.png','wb') as f:
            f.write(image)

        image = Image.open('screen.png')
        image = numpy.array(image, dtype=numpy.uint8)
        
        r,g,b,a = image[880][1525]
        if(g == 154):
            print("yes")
            t=2
            fish()
            break
        else:
            print("no")
    if t==2:
        break

if t==1:
    print("false")
else:
    print("true")
        