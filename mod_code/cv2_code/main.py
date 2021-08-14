import cv2 as cv
import numpy as np
from time import time
from .windowcapture import WindowCapture

## TODO: return the rewards and the xy coordinates from the bounding boxes


class window_det():
    def __init__(self):
        self.wincap = WindowCapture('CLAW_Arm')
        loop_time = time() # for fps
        self.reward = 0
    def iou(self,area1,area2):
        x1,y1,w1,h1 = area1
        x2,y2,w2,h2= area2
        if(x1>=x2) and (x1<=(x2+w2)) and (y1>=y2) and (y1<=y2+w2):
            return 1

    
    def ret_reward(self):
        # get an updated image of the game
        screenshot = self.wincap.get_screenshot()
        grey = cv.cvtColor(screenshot,cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(grey,220,255,cv.THRESH_BINARY_INV)
        
        contours,_ = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
        area1 = []
        area2 = []
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            if (w*h==600):
                continue
            print("Area:"+str(w*h))
            if w*h == 420:
                area1 = [x,y,w,h]
            else:
                area2 = [x,w,y,h]

            # get the min area rect
            
            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            # convert all coordinates floating point values to int
            box = np.int0(box)
            # draw a blue rectangle
            cv.drawContours(screenshot, [box], 0, (255, 0, 0),2)
        try:
            reward = self.iou(area1,area2)
        except:
            print("one or more boxes not detected")
            cv.imshow('Detection', screenshot)
            reward = 0
            return reward
        
        cv.imshow('Detection', screenshot)
        
        # uncomment to check fps
        # print('FPS {}'.format(1 / (time() - loop_time)))
        # self.loop_time = time()

        key = cv.waitKey(1)
        if key== ord('q'):
            cv.destroyAllWindows()
            
        elif key==ord('f'):
            cv.imwrite('mod_code\cv2_code\positive\{}.jpg'.format(self.loop_time),screenshot)
        return reward



if __name__ == '__main__':
    win = window_det()
    while(True):
        win.ret_reward()
    