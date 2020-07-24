import cv2
import numpy as np
import os
import math
import csv
import copy
import serial
import cv2.aruco as aruco
import time
import sys




trigger = 10

''' ############### COMMANDS FOR OPERATION ############### ''' 

'''                
                   s        - STOP BOT
                   m        - MOVE BOT
                   f        - FINISH( Finish ACK)
'''




''' SERIAL COMMUNICATION INITIALIZATION '''
ser = serial.Serial('COM6' , 9600)
ser.close()
ser.open()



''' CALCULATE (X,Y,R) CO-ORDINATES OF RED AND CENTRE WHITE CIRCLE '''
def get_Centre_Radius(img1):
   lists = [[[160 , 120 , 70] , [180 , 255 , 255]]]    #red


   mat_o = np.ones((5 , 5))
   mat_c = np.ones((20 , 20))

   #img1= cv2.cvtColor(img.copy(),cv2.COLOR_BGR2HSV)

   img0 = np.zeros(img1.shape[:2] , np.uint8)
   h , w = img1.shape[:2]
   cv2.circle(img0 , (int(w/2) , int(h/2)) , int(450) , [0 , 0 , 0] , -1)#draw circle with (x,y,r) circle for masking for removing background image colors to interfere
   cv2.circle(img0 , (int(w/2) , int(h/2)) , int(110) , [255 , 255 , 255] , -1)
   #cv2.imshow('',img0);cv2.waitKey(0);
   img = cv2.bitwise_and(img1 , img1 , mask = img0)
 #For innermost white circle

   #upper white
   l1_white = np.array([0 , 0 ,155])
   u1_white = np.array([255 , 100 , 255])
   #lower white
   l2_white = np.array([0 , 0 ,181])
   u2_white = np.array([255 , 73 , 255])

   x_w = []
   y_w = []
   r_w = [] #(x_w,y_w,r_w)---->((x,y),radius) of innermost white circle
   hsv_img = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
   conts_w , h = cv2.findContours((cv2.inRange(hsv_img , l1_white , u1_white) + cv2.inRange(hsv_img ,  l2_white , u2_white)).copy() , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)  #finding Contours of collection of white colored pixels
   for i in range(len(conts_w)):
          area_w = (cv2.contourArea(conts_w[i]))#finding area of white contours
          if ((area_w > 80) & (area_w < 120)):
                 (a , b) , c = cv2.minEnclosingCircle(conts_w[i])# Find (x,y,r) of innermost circle
                 x_w.append(round(a , 2))
                 y_w.append(round(b , 2))
                 r_w.append(round(c , 2))
   k = y_w[0]


   #m_w=cv2.circle(img1,(int(x_w[0]),int(y_w[0])) , int(r_w[0]),(255,0,0),2)
   #cv2.imshow('',g);cv2.waitKey(0);

   mask_ini = np.zeros(img.shape[:2] , np.uint8)   #Masking
   cv2.circle(mask_ini , (int(x_w[0]) , int(y_w[0])) , int(450) , [0 , 0 , 0] , -1)#draw circle with (x,y,r) circle for masking for removing background image colors to interfere
   cv2.circle(mask_ini , (int(x_w[0]) , int(y_w[0])) , int(150) , [255 , 255 , 255] , -1)
   #z=cv2.bitwise_and(img1.copy(),img1.copy(),mask=mask_ini)
   #cv2.imshow('',z);cv2.waitKey(0);

   #red and green acrylic coins
   HSV = cv2.cvtColor(cv2.bitwise_and(img1.copy() , img1.copy() , mask = mask_ini) , cv2.COLOR_BGR2HSV)# masking with original image
   c_x = []
   c_y = []
   c_r = [] #(x_w,y_w,r_w)---->((x,y),radius) of red and acrylic coins
   for (l , u) in lists :
         mask_r_g = cv2.morphologyEx(cv2.morphologyEx(cv2.inRange(HSV , np.array(l) , np.array(u)) , cv2.MORPH_OPEN , mat_o) , cv2.MORPH_CLOSE , mat_c)  #masking
         conts_r_g , _ = cv2.findContours(mask_r_g.copy() , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE) #finding Contours of collection of red and green colored pixels
         for i in range(len(conts_r_g)):
           area_r_g = (cv2.contourArea(conts_r_g[i]))# finding area of red and green contours
           if ((area_r_g > 55) & (area_r_g < 150)):
                 (x , y) , r = cv2.minEnclosingCircle(conts_r_g[i]) # Find (x,y,r) of red andgreen acrylic coins
                 c_x.append(round(x , 2))
                 c_y.append(round(y , 2))
                 c_r.append(round(r , 2))
   c_x.append(x_w[0])
   c_y.append(k)
   c_r.append(r_w[0])
   #g=cv2.circle(img1.copy(),(int(c_x[0]),int(c_y[0])) , int(c_r[0]),(255,0,0),2)
   #g=cv2.circle(g,(int(c_x[1]),int(c_y[1])) , int(c_r[1]),(255,0,0),2)
   #cv2.imshow('',g);cv2.waitKey();
   #print(len(c_x))
   return (c_x,c_y,c_r)




''' MASK FOR NODE DETECTION '''
def mask_for_nodes(img , w , h):
  y = img.copy()
  x = np.zeros(y.shape[:2] , np.uint8)
  cv2.circle(x , (int(w) , int(h)) , int(205) , [255 , 255 , 255] , -1)
  cv2.circle(x , (int(w) , int(h)) , int(155) , [0 , 0 , 0] , -1)
  z = cv2.bitwise_and(y , y , mask = x)
  return z




'''  ARUCO LIBRARY - START '''
def angle_calculate(pt1 , pt2 , trigger = 0):  # function which returns angle between two points in the range of 0-359
    angle_list_1 = list(range(359,0,- 1))
    angle_list_2 = list(range(359,0,- 1))
    angle_list_2 = angle_list_2[ - 90:] + angle_list_2[: - 90]
    x = pt2[0] - pt1[0] # unpacking tuple
    y = pt2[1] - pt1[1]
    angle = int(math.degrees(math.atan2(y , x))) #takes 2 points nad give angle with respect to horizontal axis in range(-180,180)
    if trigger == 0:
        angle = angle_list_2[angle]
    else:
        angle = angle_list_1[angle]
    return int(angle)

def detect_Aruco(img):  #returns the detected aruco list dictionary with id: corners
    aruco_list = {}
    gray = cv2.cvtColor(img , cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)   #creating aruco_dict with 5x5 bits with max 250 ids..so ids ranges from 0-249
    parameters = aruco.DetectorParameters_create()  #refer opencv page for clarification
    #lists of ids and the corners beloning to each id
    corners , ids , _ = aruco.detectMarkers(gray , aruco_dict , parameters = parameters)
    #corners is the list of corners(numpy array) of the detected markers. For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    gray = aruco.drawDetectedMarkers(gray , corners,ids)
    if len(corners):    #returns no of arucos
        for k in range(len(corners)):
            temp_1 = corners[k]
            temp_1 = temp_1[0]
            temp_2 = ids[k]
            temp_2 = temp_2[0]
            aruco_list[temp_2] = temp_1
        return (aruco_list)

def mark_Aruco(img , aruco_list):    #function to mark the centre and display the id
    key_list = aruco_list.keys()
    font = cv2.FONT_HERSHEY_SIMPLEX
    for key in key_list:
        dict_entry = aruco_list[key]    #dict_entry is a numpy array with shape (4,2)
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]#so being numpy array, addition is not list addition
    
        centre[:] = [(x / 4) for x in centre]    #finding the centre
        orient_centre = centre + 0.5
        centre = tuple(centre)  
        orient_centre = tuple((dict_entry[0] + dict_entry[1]) / 2)
        cv2.circle(img , centre , 1 , (0 , 0 , 255) , 8)
        cv2.circle(img , tuple(dict_entry[0]) , 1 , (0 , 0 , 255) , 8)
        cv2.circle(img , tuple(dict_entry[1]) , 1 , (0 , 255 , 0) , 8)
        cv2.circle(img , tuple(dict_entry[2]) , 1 , (255 , 0 , 0) , 8)
        cv2.circle(img , orient_centre , 1 , (0 , 0 , 255) , 8)
        cv2.line(img , centre , orient_centre , (255 , 0 , 0) , 4) #marking the centre of aruco
        cv2.putText(img , str(key) , (int(centre[0] + 20) , int(centre[1])) , font , 1 , (0 , 0 , 255) , 2 , cv2.LINE_AA) # displaying the idno
    return (img , centre)

''' ARUCO LIBRARY - END'''
def calculate_Robot_State(img , aruco_list):  #gives the state of the bot (centre(x), centre(y), angle)
    robot_state = {}
    key_list = aruco_list.keys()
    font = cv2.FONT_HERSHEY_SIMPLEX

    for key in key_list:
        dict_entry = aruco_list[key]
        pt1 , pt2 = tuple(dict_entry[0]) , tuple(dict_entry[1])
        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
        centre[:] = [int(x / 4) for x in centre]
        centre = tuple(centre)
        angle = angle_calculate(pt1 , pt2)
        cv2.putText(img , str(angle) , (int(centre[0] - 80) , int(centre[1])) , font , 1 , (0 , 0 , 255) , 2 , cv2.LINE_AA)
        robot_state[key] = [key , int(centre[0]) , int(centre[1]) , angle]#HOWEVER IF YOU ARE SCALING IMAGE AND ALL...THEN BETTER INVERT X AND Y...COZ THEN ONLY THE RATIO BECOMES SAME
    return (robot_state)




''' CALCULATE ARUCO ANGLE '''
def aruco_ang(ip_image):
    det_aruco_list = detect_Aruco(ip_image)
    img,center = mark_Aruco(ip_image , det_aruco_list)
    robot_state=calculate_Robot_State(img,det_aruco_list)
    #print(center)
    return (center,img)



''' IDENTIFY (X,Y) CO-ORDINATES OF NODES FOUND '''
def node_pos(img , r1 , r2 , aruco):
   font = cv2.FONT_HERSHEY_SIMPLEX
   mat_o = np.ones((5 , 5))
   mat_c = np.ones((20 , 20))

   l1_white = np.array([0 , 0 , 183])
   u1_white = np.array([255 , 72 , 255])


   l2_white = np.array([0 , 0 , 181])
   u2_white = np.array([255 , 73 , 255])
   area1 = []

   aruco_ang = node_angle(aruco[0] , aruco[1] , r1 , r2)


   #mask_ini = np.zeros(img.shape[:2],np.uint8)   #Masking
   hsv = cv2.cvtColor(img.copy() , cv2.COLOR_BGR2HSV)   #BGR2HSV
   x_w = []
   y_w = []
   r_w = []        #(x_w,y_w,r_w)---->((x,y),radius) of innermost white circle
   m_w = cv2.morphologyEx(cv2.morphologyEx(cv2.inRange(hsv , np.array(l1_white) , np.array(u1_white)) + cv2.inRange(hsv , np.array(l2_white) , np.array(u2_white)) , cv2.MORPH_OPEN , mat_o) , cv2.MORPH_CLOSE , mat_c)   #masking
   conts_w , h = cv2.findContours(m_w.copy() , cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_NONE)  #finding Contours of collection of white colored pixels
   theata_all = []
   for i in range(len(conts_w)):
      area_w = (cv2.contourArea(conts_w[i]))#finding area of white contours
      if (((area_w > 40) & (area_w < 100))):
          area1.append(conts_w[i])
          (a , b) , c = cv2.minEnclosingCircle(conts_w[i])# Find (x,y,r) of innermost circle
          x_w.append(round(a , 2))
          y_w.append(round(b , 2))
          r_w.append(round(c , 2))
          theata = node_angle(a , b , r1 , r2)
          #print(a,b,theata)
          cv2.putText(img , str(round(theata , 2)) , (int(a) , int(b)) , font , 1 , (0 , 0 , 255) , 1 , cv2.LINE_AA)
          theata_all.append(round((theata) , 2))
   #print(len(area1))
   #cv2.drawContours(img , area1 , -1 , (0 , 255 , 0) , 3)
   #cv2.imshow('' , img)
   #cv2.waitKey(0)
   #radius_x=[]
   theata_all.sort()
   theata_all.insert(0 , aruco_ang)
   return (theata_all)




''' CALCULATING ANGLE BETWEEN CITIES(NODES) AND Y AXIS '''
def node_angle(a , b , x1 , x2):
  v1 = [a-round(x1 , 2) , b-round(x2 , 2)]
  #vector1 = [targetX - gunX, targetY - gunY] # Vector of aiming of the gun at the target
  v2 = [0 , 1] #vector of X-axis
  l1 = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
  l2 = math.sqrt(v2[0] * v2[0] + v2[1] * v2[1])
  ang = (math.degrees(math.acos((v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2))))
  if(a >= 320):
      return (360 - ang)
  else:
      return(ang)



''' ANGLE DETECTION '''
def calAngle(r , w , g):  # r ===> red , w ===> white , g ===> green
    ang = math.degrees(math.atan2(g[1]-w[1] , g[0]-w[0])-math.atan2(r[1]-w[1] , r[0]-w[0])) #basic formula for finding angle
    if (ang < 0):
       ang1 = (ang + 360)
    else:
       ang1 = ang
    if(ang1 > 180):
        return(360 - ang1)
    else:
        return(ang1)



''' SEND SERIAL DATA '''
def send(data):
    ser.write(str.encode(data))
    #print(data)
    #ser.write(str.encode(chr(0x60)))
    #ser.close()



''' CONDITION FOR SENDING SERIAL DATA  - FOR REACHING MEDICAL AID '''
def red_ang(ang_wrt_r):
  if( - trigger <= ang_wrt_r <= trigger):
    for i in range(0,75):  
      send('s')
      time.sleep(0.01)
    return (1)
  else :
    send('m')
    return (0)



''' CONDITION FOR SENDING SERIAL DATA - FOR REACHING FINISH POINT ''' 
def aruco_fin_angle(ang_ini):
  if( - 10 <= ang_ini <= 10):
      for i in range(0,75):  
         send('f')
         time.sleep(0.01)
      time.sleep(7)   
      return (1)
  else:
      send('m')
      return (0)



''' FOR TRIGGERING BOT OPERATIONS AND OTHER FUNCTIONS'''
def main():
    ################################################################
    ## variable declarations
    ################################################################
    #i = 1
    ## reading in video 
    cap = cv2.VideoCapture(1) #if you have a webcam on your system, then change 0 to 1
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    fps1 = fps

    x = [0,0]
    y = [0,0]
    r = [0,0]
    ## setting the video counter to frame sequence
    cap.set(3 , 640)
    cap.set(4 , 480)

    ## reading in the frame
    ret , frame = cap.read()
    ## verifying frame has content
    print(frame.shape)

    for f in range(0 , int(fps1)):
        _ , frame = cap.read()
        #cv2.imshow('',frame);cv2.waitKey(0);
        c_x , c_y , c_r = get_Centre_Radius(frame)
        for l1 in range(len(c_x)):
           x[l1] += c_x[l1]
           y[l1] += c_y[l1]
           r[l1] += c_r[l1]
    for d1 in range(len(c_x)):
      x[d1] = x[d1]/(fps1)
      y[d1] = y[d1]/(fps1)
      r[d1] = r[d1]/(fps1)

    circle_x = x
    circle_y = y
    #print(circle_x[0],circle_y[0])
    #f = cv2.circle(frame,(int(circle_x[0]),int(circle_y[0])) , int((circle_r[0])),(255,0,0),2)
    #f = cv2.circle(f,(int(circle_x[1]),int(circle_y[1])) , int((circle_r[1])),(255,0,0),2)
    #cv2.imshow('hjh',f);cv2.waitKey(0);

    frame1 = mask_for_nodes(frame , circle_x[1] , circle_y[1])
    aruco_pt , img = aruco_ang(frame)
    final_pt = aruco_pt
    list1 = node_pos(frame1 , circle_x[1] , circle_y[1] , aruco_pt)

    ang_fin = []
    for f1 in range(len(circle_x)-1):
        ang_fin.append(node_angle(circle_x[f1] , circle_y[f1] , circle_x[1] , circle_y[1]))

    for f1 in range(len(ang_fin)):
        if((list1[0]-9) <= ang_fin[f1] <= (list1[0]+9)):
            print('Medical Aid' , '-----> 1')
        elif((list1[1]-9) <= ang_fin[f1] <= (list1[1]+9)):
            print('Medical Aid' , '-----> 2')
        elif((list1[2]-9) <= ang_fin[f1] <= (list1[2]+9)):
            print('Medical Aid' , '-----> 3')
        elif((list1[3]-9) <= ang_fin[f1] <= (list1[3]+9)):
            print('Medical Aid' , '-----> 4')
        elif((list1[4]-9) <= ang_fin[f1] <= (list1[4]+9)):
            print('Medical Aid' , '-----> 5')
        elif((list1[5]-9) <= ang_fin[f1] <= (list1[5]+9)):
            print('Medical Aid' , '-----> 6')
        elif((list1[6]-9) <= ang_fin[f1] <= (list1[6]+9)):
            print('Medical Aid' , '-----> 7')
        elif((list1[7]-9) <= ang_fin[f1] <= (list1[7]+9)):
            print('Medical Aid' , '-----> 8')
        elif((list1[8]-9) <= ang_fin[f1] <= (list1[8]+9)):
            print('Medical Aid' , '-----> 9')
        else:
            print('Place your medical Aid in the correct city(node)')

    # print(time.time()-start)

    r = 0
    a_f = 0
    #input1 = input(" Type 'i' or 'I' to initiate bot operation : ")

    #if((input1 == 'i') | (input1 == 'i')):
    while(ret):
          ret, frame = cap.read()
          ## display to see if the frame is correct
          cv2.imshow("window" , frame)
          cv2.waitKey(int(1000/fps))
          aruco_pt , img = aruco_ang(frame)
          cv2.imshow("aruco" , img)
          cv2.waitKey(int(1000/fps))
          #cv2.imshow('image',img);cv2.waitKey(0)
          if(r == 0):
              ang_wrt_r = calAngle((aruco_pt[0] , aruco_pt[1]) , (circle_x[1] , circle_y[1]) ,
                                 (circle_x[0] , circle_y[0]))  #(aruco , white , red)
              #print('red')
              r = red_ang(ang_wrt_r)

          if ((r != 0) & (a_f == 0)):
             ang_ini = calAngle((aruco_pt[0] , aruco_pt[1]) , (circle_x[1] , circle_y[1]) ,
                                (final_pt[0] , final_pt[1])) #(aruco , white , g1)
             #print('Finishing pt')
             a_f = aruco_fin_angle(ang_ini)
             
          if ((r != 0) & (a_f != 0)) :
              sys.exit()


''' TRIGGERING MAIN FUNCTION'''
if __name__ == '__main__':
    main()
