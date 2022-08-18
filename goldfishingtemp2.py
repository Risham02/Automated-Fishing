# (hMin = 38 , sMin = 24, vMin = 88)
# (hMax = 67 , sMax = 48, vMax = 107)

#left = 240, right = 2085

import threading
from multiprocessing import Process
from ppadb.client import Client
import cv2
import numpy
import time


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

def is_fishing():
    print("checking if already fishing")
    file = open("screen.dump", "rb")
    byte = file.read((876*2400+1652)*4+16)
    color = []
    i = 0
    while i<4:
        byte = file.read(1)
        color.append(int(byte.hex(),16))
        i+=1
    print(color)
    if color[2] == 90:
        print("yes")
        return 1
    else:
        print("no")
        return 0

def low_concentration():
    print("checking concentration")
    file = open("screen.dump", "rb")
    byte = file.read((990*2400+1490)*4+16)
    color = []
    i = 0
    while i<4:
        byte = file.read(1)
        color.append(int(byte.hex(),16))
        i+=1
    if color[2] != 133 and is_fishing() == 1:
        device.shell('input tap 2160 400')
        print("using potion")
        print(color)
        time.sleep(0.5)

def reel_in():
    print("reel in")
    device.shell('input tap 2170 560')

def fish(point):
    print("fishing...")
    t1_start = time.perf_counter()
    device.shell('input tap %i %i' %(point[0],point[1]))
    time.sleep(0.1)
    device.shell('input tap 2200 900')
    time.sleep(0.2)
    

    while True:
        device.shell('screencap /sdcard/screen.dump')
        device.pull("/sdcard/screen.dump",'screen.dump')

        if(is_fishing() == 0):
            break
        
        reel_in()
        device.shell('input tap %i %i' %(point[0],point[1]))
        low_concentration()
        device.shell('input tap %i %i' %(point[0],point[1]))

def movingleft():
    print("move left")
    device.shell('input swipe 450 890 20 850 18000')
    print("sub move left stopped")

def movingright():
    print("move right")
    device.shell('input swipe 250 890 1000 750 12000')
    print("sub move right stopped")

def ley(flag):
    print("to ley")
    time.sleep(0.1)
    if(flag == 0):
        device.shell('input tap 170 80')
        device.shell('input tap 730 940')
        device.shell('input tap 1170 865')
        device.shell('input tap 1090 700')
        device.shell('input swipe 750 600 1150 600 300')
        device.shell('input swipe 310 950 310 700 500')
    else:
        device.shell('input tap 170 80')
        device.shell('input tap 730 940')
        device.shell('input tap 1170 230')
        device.shell('input tap 1090 700')
        device.shell('input swipe 750 600 1150 600 300')
        device.shell('input swipe 310 950 310 700 500')

def interrupt():
    print("interrupted")
    device.shell('input tap 1280 640')

def emain():
    # updated image of screen
    # screenshot = device.screencap()
    # with open('screen.jpg','wb') as f:
    #     f.write(screenshot)
    # screenshot = Image.open('screen.jpg')
    # screenshot = numpy.array(screenshot)

    device.shell('screencap /sdcard/screen.dump')
    device.pull("/sdcard/screen.dump",'screen.dump')

    file = open("screen.dump", "rb")
    byte = file.read()
    byte = numpy.array(list(byte))
    byte = byte[16:]
    byte = numpy.array(byte)
    screenshot = byte.reshape(-1,2400,4)
    screenshot = numpy.uint8(screenshot)

    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2BGR)

    mask = numpy.zeros(screenshot.shape[:2], dtype="uint8")
    cv2.rectangle(mask, (235, 150), (2085,1080), 255, -1)
    mask1 = numpy.ones(screenshot.shape[:2], dtype="uint8")
    cv2.circle(mask1,(300,871),175,0,-1)
    mask = cv2.bitwise_and(mask,mask1)
    screenshot = cv2.bitwise_and(screenshot, screenshot, mask=mask)
    
    screenshot = cv2.blur(screenshot,(12,12)) 
    # converting to hsv color scheme
    hsv_frame = cv2.cvtColor(screenshot,cv2.COLOR_BGR2HSV)

    # the min and max values for hsv filter for shimmering
    low_shimmering = numpy.array([39,19,73])
    high_shimmering = numpy.array([75,41,121])

    # filtering the range values
    shimmering_mask = cv2.inRange(hsv_frame,low_shimmering,high_shimmering)

    # shimmering_mask = cv2.bitwise_not(shimmering_mask)    
    # finding list of rectangles of most likely places where image is found
    #rectangles = find_rectangles(shimmering_mask,shimmering_image)

    # perform a series of erosions and dilations to remove
    # any small blobs of noise from the thresholded image
    shimmering_mask = cv2.erode(shimmering_mask, None, iterations=1)
    shimmering_mask = cv2.dilate(shimmering_mask, None, iterations=1)

    # drawing the rectangles
    #shimmering_mask = draw_rectangles(shimmering_mask,rectangles)
    
    keypoints = detector.detect(shimmering_mask)
    keypoints2 = get_click_points(keypoints)
    print(len(keypoints2))
    print(keypoints2)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(shimmering_mask, keypoints, numpy.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    cv2.imshow('Inrange', im_with_keypoints)
    

    # press 'q' with the output window focused to exit
    # wait 1 ms every loop to process key presses
    if cv2.waitKey(1) == ord('q'):
        cv2.destroyAllWindows()

    return keypoints2

if __name__ == '__main__':

    process_list = []
    process_list.append(Process(target=movingright))
    process_list[0].start()
    
    flag = 0
    time_start = time.time()
    
    while True:

        keypoints2 = emain()

        #choose_valid_point(keypoints2)
        if len(keypoints2)>0:
            time_pause = time.time()
            # interrupt()
            print("stop move")
            process_list[0].terminate()
            interrupt()
            print("move stopped")
            process_list.pop(0)
            print("process list updated")
            
            # interrupt()  
            count = 0
            temp = 0
            keypoints2 = emain()
            if len(keypoints2)>0:
                count+=1
                print("equip fishing rod")
                device.shell('input tap 2130 390')
                while len(keypoints2)>0:
                    for i in range(0,len(keypoints2)):
                        fish(keypoints2[i])    
                    keypoints2 = emain()   
                    if(count>4):
                        temp = 1
                        break
                print("mount")
                device.shell('input tap 2315 415')
            if(flag == 0):
                process_list.append(Process(target=movingright))
                process_list[0].start()
                print("main.start right")
            else:
                process_list.append(Process(target=movingleft))
                process_list[0].start()
                print("main.start left")
            time_start = time_start+(time.time()-time_pause)
            if(temp == 1):
                time.sleep(1)
        print(time.time()-time_start)
        if(flag == 0 and time.time()-time_start>=11):
            print("Time exceeded right")
            process_list[0].terminate()
            print("move right stopped")
            process_list.pop(0)
            print("process list updated")
            
            # interrupt()
            ley(flag)
            print("moved to ley")
            flag +=1
            process_list.append(Process(target=movingleft))
            print("start moving left")
            process_list[0].start()
            print("started moving left")
            time_start = time.time()
        elif(flag == 1 and time.time()-time_start>=19):
            print("Time limit exceeded left")
            process_list[0].terminate()
            print("move left stopped")
            process_list.pop(0)
            print("process list updated")
            
            # interrupt()
            ley(flag)
            print("Moved to ley")
            flag -=1
            process_list.append(Process(target=movingright))
            print("start moving right")
            process_list[0].start()
            print("Started moving right")
            time_start = time.time()





