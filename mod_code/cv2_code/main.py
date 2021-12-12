import cv2 as cv
import numpy as np
import time
from time import time as tm
import os
from pygame import Rect
from windowcapture import WindowCapture

## TODO: return the rewards and the xy coordinates from the bounding boxes


class window_det():
    def __init__(self):
        self.wincap = WindowCapture('CLAW_Arm')
        self.loop_time = tm() # for fps
        self.reward = 0
        self.count=264
    def iou(self,rect1,rect2):
        x = 0
        ((x1,y1),(w1,h1),(a1)) = rect1
        ((x2,y2),(w2,h2),(a2)) = rect2
        if (x1>=x2-w2)and (x1<=x2+w2) and (y1>=y2-h2) and (y1<=y2+h2):
             x =1
        return x      
    def ret_reward(self):
        # get an updated image of the game
        screenshot = self.wincap.get_screenshot()
        # grey = cv.cvtColor(screenshot,cv.COLOR_BGR2GRAY)
        # ret,thresh = cv.threshold(grey,220,255,cv.THRESH_BINARY_INV)
        
        # contours,_ = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        
        # for c in contours:
        #     x, y, w, h = cv.boundingRect(c)
        #     if (w*h==600):
        #         continue
        #     if w*h == 420:
        #         rect1 = cv.minAreaRect(c)
        #         box = cv.boxPoints(rect1)
        #         # convert all coordinates floating point values to int
        #         box = np.int0(box)
        #         # draw a blue rectangle
        #         cv.drawContours(screenshot, [box], 0, (255, 0, 0),2)
       
        #     else:
                
        #         rect2 = cv.minAreaRect(c)
        #         box = cv.boxPoints(rect2)
        #         # convert all coordinates floating point values to int
        #         box = np.int0(box)
        #         # draw a blue rectangle
        #         cv.drawContours(screenshot, [box], 0, (255, 0, 0),2)
       

        # try:
        #     reward = self.iou(rect1,rect2)
        # except:
        #     print("one or more boxes not detected")
        #     cv.imshow('Detection', screenshot)
        #     reward = 0
        #     return reward
        
        cv.imshow('Detection', screenshot)
        
        # # uncomment to check fps
        # print('FPS {}'.format(1 / (time() - self.loop_time)))
        # self.loop_time = time()
        self.count+=1

        if not cv.imwrite('./mod_code/Train/{}.jpg'.format(self.count),screenshot):
            raise Exception("Not saved")
   

        key = cv.waitKey(1)
        if key== ord('q'):
            cv.destroyAllWindows()
            
        return 0



if __name__ == '__main__':
    win = window_det()
    while(True):
        win.ret_reward()
        time.sleep(0.7)