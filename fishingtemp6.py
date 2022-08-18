from tkinter import TRUE
from ppadb.client import Client
import cv2
import numpy
import time
from PIL import Image

adb = Client(host='127.0.0.1', port=5037)
devices = adb.devices()
if len(devices) == 0:
    print('no device attached')
    quit()

device = devices[0]


# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = 0
params.blobColor = 1

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area.
params.filterByArea = True
params.minArea = 500
params.maxArea = 5000000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0

params.minDistBetweenBlobs = 150

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

# function to get points at center of blob
def get_click_points(keypoints):
    
    points = []
    
    # Loop over all the keypoints
    for keyPoint in keypoints:
        x = int(keyPoint.pt[0])
        y = int(keyPoint.pt[1])
        # saving the points
        points.append((x,y))
    
    return points

# def choose_valid_point(points):
#     print("choose_valid_point")
#     # for point in points:
#     #     print("point loop")
#     if len(points)>0:
#         fish(points[0])
#         # if check_valid_point(point) == 1:
#         #     fish(point)
#         #     break
        

# def check_valid_point(point):
#     print("checking point")
#     device.shell('input tap %i %i' %(point[0],point[1]))
#     screenshot = device.screencap()
#     with open('screen.png','wb') as f:
#         f.write(screenshot)
#     image = Image.open('screen.png')
#     image = numpy.array(image, dtype=numpy.uint8)
#     color = image[920, 1645]
#     if color[0] == 218:
#         print("valid point")
#         return 1
#     else:
#         print("invalid point")
#         return 0

def is_fishing():
    print("checking if already fishing")
    image = Image.open('screen.jpg')
    image = numpy.array(image, dtype=numpy.uint8)
    color = image[876, 1652]
    if color[2] == 90:
        print("yes")
        return 1
    else:
        print("no")
        return 0

def low_concentration():
    print("checking concentration")
    image = Image.open('screen.jpg')
    image = numpy.array(image, dtype=numpy.uint8)
    color1 = image[990, 1490]
    color2 = image[980, 1610]
    if color1[2] != 133 and is_fishing() == 1:
        device.shell('input tap 2160 400')
        print("using potion")
        print(color1)
        print(color2)
        time.sleep(0.5)
        

def reel_in():
    print("reel in")
    device.shell('input tap 2170 560')

def fish(point):
    print("fishing...")
    device.shell('input tap %i %i' %(point[0],point[1]))
    time.sleep(0.1)
    device.shell('input tap 2200 900')
    time.sleep(0.2)
    screenshot = device.screencap()
    with open('screen.jpg','wb') as f:
        f.write(screenshot)
    print("fishing screenshot1")
    time.sleep(1)
    if(is_fishing() == 0):
        return
    while TRUE:
        reel_in()
        device.shell('input tap %i %i' %(point[0],point[1]))
        low_concentration()
        device.shell('input tap %i %i' %(point[0],point[1]))
        time.sleep(1)
        screenshot = device.screencap()
        with open('screen.jpg','wb') as f:
            f.write(screenshot)
        print("fishing screenshot")
        if(is_fishing() == 0):
            break

while True:

    # updated image of screen
    screenshot = device.screencap()
    with open('screen.jpg','wb') as f:
        f.write(screenshot)
    screenshot = Image.open('screen.jpg')
    screenshot = numpy.array(screenshot)
    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2BGR)

    screenshot = cv2.blur(screenshot,(12,12)) 
    # converting to hsv color scheme
    hsv_frame = cv2.cvtColor(screenshot,cv2.COLOR_BGR2HSV)

    # the min and max values for hsv filter for rippling
    low_rippling = numpy.array([83,79,113])
    high_rippling = numpy.array([98,135,135])

    # filtering the range values
    rippling_mask = cv2.inRange(hsv_frame,low_rippling,high_rippling)

    # rippling_mask = cv2.bitwise_not(rippling_mask)    
    # finding list of rectangles of most likely places where image is found
    #rectangles = find_rectangles(rippling_mask,rippling_image)

    # perform a series of erosions and dilations to remove
    # any small blobs of noise from the thresholded image
    rippling_mask = cv2.erode(rippling_mask, None, iterations=1)
    rippling_mask = cv2.dilate(rippling_mask, None, iterations=1)

    # drawing the rectangles
    #rippling_mask = draw_rectangles(rippling_mask,rectangles)
    
    keypoints = detector.detect(rippling_mask)
    keypoints2 = get_click_points(keypoints)
    print(len(keypoints2))
    print(keypoints2)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(rippling_mask, keypoints, numpy.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow('Inrange', im_with_keypoints)
    #choose_valid_point(keypoints2)
    if len(keypoints2)>0:
        if len(keypoints2)<4:
            for i in range(0,len(keypoints2)):
                fish(keypoints2[i])
        else:
            for i in range(0,4):
                fish(keypoints2[i])


    # press 'q' with the output window focused to exit
    # wait 1 ms every loop to process key presses
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()
        break

print('Done')