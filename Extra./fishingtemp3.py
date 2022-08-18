from ppadb.client import Client
import time
import cv2
import numpy

adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()
if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]

image = device.screencap()
with open('screen.png','wb') as f:
    f.write(image)

print("a")

haystack_image = cv2.imread('screen.png',cv2.IMREAD_UNCHANGED)
print("b")
needle_image = cv2.imread('images/fishingspot2pier.png',cv2.IMREAD_UNCHANGED)
print("c")
result = cv2.matchTemplate(haystack_image,needle_image,cv2.TM_SQDIFF)
print("d")
print("f")
min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(result)

print('best match top left pos: %s'%str(max_loc))
print('best match confidence: %s'%max_val)

needle_w = needle_image.shape[1]
needle_h = needle_image.shape[0]

top_left = max_loc
bottom_right = (top_left[0]+needle_w, top_left[1]+needle_h)

cv2.rectangle(haystack_image, top_left, bottom_right, color=(0,255,0), thickness=2, lineType=cv2.LINE_4)

cv2.imshow('Result', haystack_image)
cv2.waitKey(0)