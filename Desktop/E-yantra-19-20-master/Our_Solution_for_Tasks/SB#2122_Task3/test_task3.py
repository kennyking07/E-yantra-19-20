###############################################################################
## Author: Team Supply Bot
## Edition: eYRC 2019-20
## Instructions: Do Not modify the basic skeletal structure of given APIs!!!
###############################################################################


######################
## Essential libraries
######################
import cv2
import numpy as np
import os
import math
import csv
import copy


############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################


def get_Centre_Radius(img):
   lists = [[[160,120,70],[180,255,255]],     #red
            [[ 36, 25,25],[ 46,255,255]]]     #green
   mat_o = np.ones((5,5))
   mat_c = np.ones((20,20))
  
 #For innermost white circle

   #upper white
   l1_white = np.array([0,0,155])
   u1_white = np.array([255,100,255])
   #lower white
   l2_white = np.array([0,0,181])
   u2_white = np.array([255,73,255])

   mask_ini = np.zeros(img.shape[:2],np.uint8)   #Masking    
   hsv = cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)   #BGR2HSV
   x_w = []; y_w = []; r_w = [] #(x_w,y_w,r_w)---->((x,y),radius) of innermost white circle
   m_w = cv2.morphologyEx(cv2.morphologyEx(cv2.inRange(hsv,np.array(l1_white),np.array(u1_white)) + cv2.inRange(hsv,np.array(l2_white),np.array(u2_white)),cv2.MORPH_OPEN,mat_o),cv2.MORPH_CLOSE,mat_c)   #masking
   conts_w,h = cv2.findContours(m_w.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)  #finding Contours of collection of white colored pixels
   for i in range(len(conts_w)):            
          area_w = (cv2.contourArea(conts_w[i]))#finding area of white contours
          if ((area_w > 80) & (area_w < 200)):
                 (a,b),c = cv2.minEnclosingCircle(conts_w[i])# Find (x,y,r) of innermost circle
                 x_w.append(round(a,2));y_w.append(round(b,2));r_w.append(round(c,2))
   k = y_w[0]
   cv2.circle(m_w,(int(x_w[0]),int(y_w[0])),int(150),[255,255,255],-1)#draw circle with (x,y,r) circle for masking for removing background image colors to interfere 
   

   #red and green acrylic coins
   HSV = cv2.cvtColor(cv2.bitwise_and(img.copy(),img.copy(),mask=m_w.copy()),cv2.COLOR_BGR2HSV)# masking with original image
   c_x = []; c_y = []; c_r = [] #(x_w,y_w,r_w)---->((x,y),radius) of red and acrylic coins
   for (l,u) in lists :
         mask_r_g = cv2.morphologyEx(cv2.morphologyEx(cv2.inRange(HSV,np.array(l),np.array(u)),cv2.MORPH_OPEN,mat_o),cv2.MORPH_CLOSE,mat_c)  #masking
         conts_r_g,_ = cv2.findContours(mask_r_g.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #finding Contours of collection of red and green colored pixels
         for i in range(len(conts_r_g)):          
           area_r_g = (cv2.contourArea(conts_r_g[i]))# finding area of red and green contours
           if ((area_r_g > 70) & (area_r_g < 200)): 
                 (x,y),r = cv2.minEnclosingCircle(conts_r_g[i]) # Find (x,y,r) of red andgreen acrylic coins
                 c_x.append(round(x,2));c_y.append(round(y,2));c_r.append(round(r))          
   c_x.append(x_w[0]);c_y.append(k);c_r.append(r_w[0])
   #print(len(c_x))
   return (c_x,c_y,c_r)  

def calAngle(r,w,g):  # r ===> red , w ===> white , g ===> green
   ang=math.degrees(math.atan2(g[1]-w[1],g[0]-w[0])-math.atan2(r[1]-w[1],r[0]-w[0])) #basic formula for finding angle 
   if ang<0:
    return(ang+360)
   else:
    return(ang)

def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    op_image = ip_image
    cx,cy,cr = get_Centre_Radius(ip_image) # to get center and radius of red, green and white circles 
    ip_image = cv2.circle(ip_image,(int(cx[0]),int(cy[0])) , int((cr[0]+cr[1])/2),(255,0,0),2) #draw circle around red circle in blue colour 
    ip_image = cv2.circle(ip_image,(int(cx[1]),int(cy[1])) , int((cr[1]+cr[0])/2),(255,0,0),2) #draw circle around red circle in blue colour
    ang = calAngle((cx[1],cy[1]),(cx[2],cy[2]),(cx[0],cy[0])) #calling  function
    if(ang>180):
     ang=(360-ang)
    else:
     ang 
    op_image = cv2.putText(ip_image, "Angle : " +str(round(ang,2)), (60,60), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0,255), 1, cv2.LINE_AA) #put text in frame 
    return op_image

def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(0) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## setting the video counter to frame sequence
    cap.set(3, 640)
    cap.set(4, 480)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    while(ret):
        if(i <= 1720):
          ret, frame = cap.read()
          ## display to see if the frame is correct
          cv2.imshow("window", frame)
          cv2.waitKey(int(1000/fps));
          ## calling the algorithm function
          op_image = process(frame)
          cv2.imwrite("SB#2122_task3I.jpg",op_image)
          i=i+1
       


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
