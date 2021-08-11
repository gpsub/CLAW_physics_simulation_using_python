import cv2 as cv
import numpy as np
import os
from time import time
from windowcapture import WindowCapture


wincap = WindowCapture('CLAW_Arm')



loop_time = time() # for fps
while(True):

    # get an updated image of the game
    screenshot = wincap.get_screenshot()
    grey = cv.cvtColor(screenshot,cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(grey,220,255,cv.THRESH_BINARY_INV)
    cv.imshow('thresholded',thresh)
    contours,_ = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    # we will send the thresholded image to 
    # cv.drawContours(screenshot, contours,-1,(250,230,230),2)
    for c in contours:
        x, y, w, h = cv.boundingRect(c)
    
        # get the min area rect
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        # convert all coordinates floating point values to int
        box = np.int0(box)
        # draw a blue rectangle
        cv.drawContours(screenshot, [box], 0, (255, 0, 0),2)


    cv.imshow('Detection', screenshot)
     
    # uncomment to check fps
    # print('FPS {}'.format(1 / (time() - loop_time)))
    # loop_time = time()

    key = cv.waitKey(1)
    if key== ord('q'):
        cv.destroyAllWindows()
        break
    elif key==ord('f'):
        cv.imwrite('mod_code\cv2_code\positive\{}.jpg'.format(loop_time),screenshot)


print('Done.')