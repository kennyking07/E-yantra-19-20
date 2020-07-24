import cv2
import numpy as np
import os

def partA():
    cap=cv2.VideoCapture('../videos/RoseBloom.mp4');
    x=1;
    fps=25
    frame_at=6
    y=fps*frame_at
    frame=[]
    while(1):
        ret,frame=cap.read()
        if ret:
           name='../Generated/frame_as_6.jpg'
           if x==y:
             cv2.imwrite(name,frame)
           x+=1
        else:
          break

    cap.release() 
    cv2.destroyAllWindows() 

def partB():
    img=cv2.imread('../Generated/frame_as_6.jpg')
    img[:,:,1]=0
    img[:,:,0]=0
    cv2.imwrite('../Generated/frame_as_6_red.jpg',img);

partA()
partB()
