from ppadb.client import Client
import cv2
import numpy
from time import time
from PIL import Image
from vision import Vision


adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()
if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]

vision_fishingspotrippling = Vision('images/fishingspot2pier.png')

loop_time = time()
while True:

    # updated image of screen
    screenshot = device.screencap()
    with open('screen.png','wb') as f:
        f.write(screenshot)
    screenshot = Image.open('screen.png')
    screenshot = numpy.array(screenshot)
    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2BGR)

    # do onject detection
    rectangles = vision_fishingspotrippling.find(screenshot, 0.5)

    # draw the detection results on the original image
    output_image = vision_fishingspotrippling.draw_rectangles(screenshot,rectangles)

    # display processed image
    cv2.imshow('Matches', output_image)

    # debug the loop rate
    print('FPS {}'.format(1/(time()-loop_time)))
    loop_time = time()

    # press 'q' with the output window focused to exit
    # wait 1 ms every loop to process key presses
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break

print('Done')