import cv2 as cv
import numpy as np
from PIL import Image
from keras import models
import os
import tensorflow as tf
model = models.load_model('weights/')   
video = cv.VideoCapture(1)

while True: 
    _,frame = video.read()
    im = Image.fromarray(frame,'RGB')
    im = im.resize((300,300))
    img_array = np.array(im)
    img_array = np.expand_dims(img_array,axis=0)
    prediction = model.predict(img_array)
    num = np.argmax(prediction,axis=1)
    prob = np.max(prediction,axis=1)
    if num[0]==0:
        print('Cardboard '+str(prob))
    elif num[0]==1:
        print('glass'+str(prob))
    elif num[0]==2:
        print('metal'+str(prob))
    elif num[0]==3:
        print('paper'+str(prob))
    elif num[0]==4:
        print('plastic'+str(prob))
    else:
        print("trash"+str(prob))
    cv.imshow("Prediction",frame)
    key=cv.waitKey(1)
    if key == ord('q'):
                break
video.release()
cv.destroyAllWindows()

