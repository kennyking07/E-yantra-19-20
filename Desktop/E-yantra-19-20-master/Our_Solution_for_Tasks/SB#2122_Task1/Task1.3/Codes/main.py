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
import cv2.aruco as aruco
from aruco_lib import *
import copy



########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Videos'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




def sharpen(img):
    mat =np.array([0, -0.5, 0, -0.5, 3, -0.5, 0, -0.5, 0]).reshape((3, 3, 1))
    img_s = cv2.filter2D(img, -1, mat)
    return (img_s)

def fna(i_img, q=60):
   h,w,c  = i_img.shape
   img1 = cv2.copyMakeBorder(i_img,q,q,q,q, cv2.BORDER_WRAP)
   img2 = cv2.GaussianBlur(img1, (2*q+1, 2*q+1),-1)[q:-q,q:-q]
   i=0;val=0;j=0
   x1=np.zeros(i_img.shape[:2]);x2=x1;
   for i in range (x1.shape[0]):
           if(i<=h)&(val<=h):
               x1[i]=val
               x1[h-i-1]=val
               i+=1;val+=1
   x1=(h-x1-1);val=0;
   for j in range (x1.shape[1]):
       if(j<=w)&(val<=w):
             x2[:,j]=val
             x2[:,w-j-1]=val
             j+=1;val+=1
   x2=2*(w-x2-1)
   mat_1D=np.dstack([x1,x2]).min(-1)
   mat_3D=np.dstack([mat_1D,mat_1D,mat_1D])
   s= np.minimum(np.float32(mat_3D)/q,1.0)
   return (i_img*s+img2*(1-s))    


## REFERENCES mentioned below
def PSF(ang, q):
    p=20
    mat = (np.float32)(np.ones((1, q)))
    cos,sin=np.cos(ang),np.sin(ang)
    M = (np.float32)([[cos, -sin,0], [sin, cos,0]])
    p1 = (int)(p/2)
    M[:,2] = (p1, p1) - np.dot(M[:,:2], ((q-1)*0.5, 0))
    mat = cv2.warpAffine(mat, M, (p,p), flags=cv2.INTER_CUBIC)
    return (mat)

def split(img):
   img1=img.copy();img2=img.copy();
   #RED_channel
   img[:,:,1]=0;img[:,:,0]=0;
   r = np.delete((img), 0, 2)
   r1 = np.delete((r), 0, 2);r1=np.squeeze(r1,axis=2);
   #BLUE_channel
   img1[:,:,1]=0;img1[:,:,2]=0;
   b = np.delete((img1), 1, 2)
   b1 = np.delete((b), 1, 2);b1=np.squeeze(b1,axis=2)
   img2[:,:,2]=0;img2[:,:,0]=0;
   #GREEN_channel
   g = np.delete((img2), 0, 2)
   g1 = np.delete((g), 1, 2);g1=np.squeeze(g1,axis=2)
   im=[np.float32(b1)/255.0,np.float32(g1)/255.0,np.float32(r1)/255.0]
   return (im)

def join(res):
    p=np.zeros(res[0].shape,res[0].dtype);
    b_3d = np.dstack([res[0],p,p])
    g_3d=np.dstack([p,res[1],p])
    r_3d=np.dstack([p,p,res[2]])
    bgr=(b_3d+g_3d+r_3d);
    return (bgr)


## aruco_detection.py code is"modified" and used as a function here...
def aruco_detector(frame):
    robot_state=0
    det_aruco_list = {}
    det_aruco_list = detect_Aruco(frame)
    img=[];x=[];
    if det_aruco_list:
      img = mark_Aruco(frame,det_aruco_list)
      img = cv2.addWeighted(img, .9,np.ones(img.shape, img.dtype), .1, 30)
      robot_state = calculate_Robot_State(img,det_aruco_list)
      x=np.array(tuple(robot_state.values())).flatten()
    return(img,x)


############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    ###########################
    id_list = [];f_img=[]
    frame = ip_image[0:740, 0:1292]     
    frame_con=cv2.addWeighted(frame, 2.2, np.zeros(frame.shape, frame.dtype), 0, -60)
    frame_con=fna(frame_con)
    im=split(frame_con);
    for i in range(0,3):
      f_img.append(cv2.dft(np.array(im[i]), flags=cv2.DFT_COMPLEX_OUTPUT))
    angle=np.deg2rad(90);
    psf_length=19;
    SNR=10**(-0.1*18)
    psf_var= PSF(angle, psf_length)
    psf_var = psf_var/psf_var.sum()
    i=0; temp=[]
    for i in range(0,3):
        mask=np.zeros_like(im[i])
        h,w = psf_var.shape
        mask[:h, :w] = psf_var
        f_psf_var = cv2.dft(mask, flags=cv2.DFT_COMPLEX_OUTPUT, nonzeroRows = h)
        psf1 = (f_psf_var**2).sum(-1)
        iff_psf_var=(f_psf_var / (psf1 + SNR)[...,np.newaxis])
        f_temp=(cv2.mulSpectrums(f_img[i], iff_psf_var, 0))
        temp1=(cv2.idft(f_temp, flags=cv2.DFT_SCALE))
        temp1 = cv2.magnitude(temp1[:,:,0],temp1[:,:,1])
        temp.append(temp1)
    bgr=join(temp)
    bgr=np.multiply(255,bgr)
    bgr = bgr.astype(np.uint8)
    bgr=np.clip(bgr,0,255);
    ip_image,id_list=aruco_detector(bgr);
    ip_image = cv2.fastNlMeansDenoisingColored(ip_image, None, 6, 4, 15, 45)
    ip_image= cv2.addWeighted(ip_image, 1.5, np.ones(ip_image.shape, ip_image.dtype), 0, -40)
    ip_image= cv2.copyMakeBorder(ip_image,0,100,0,308,cv2.BORDER_CONSTANT,value=[255,255,255]);
    ip_image=sharpen(ip_image)

    cv2.imwrite((generated_folder_path+"/"+"aruco_with_id.png"),ip_image)
    return (ip_image, id_list)

############################REFERENCES##############################
##  https://fossies.org/linux/opencv/samples/python/deconvolution.py
####################################################################

####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main(val):
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## reading in video 
    cap = cv2.VideoCapture(images_folder_path+"/"+"aruco_bot.mp4")
    ## getting the frames per second value of input video
    fps = cap.get(cv2.CAP_PROP_FPS)
    ## getting the frame sequence
    frame_seq = int(val)*fps
    ## setting the video counter to frame sequence
    cap.set(1,frame_seq)
    ## reading in the frame
    ret, frame = cap.read()
    ## verifying frame has content
    print(frame.shape)
    ## display to see if the frame is correct
    cv2.imshow("window", frame)
    cv2.waitKey(0);
    ## calling the algorithm function
    op_image, aruco_info = process(frame)
    ## saving the output in  a list variable
    line = [str(i), "Aruco_bot.jpg" , str(aruco_info[0]), str(aruco_info[3])]
    ## incrementing counter variable
    i+=1
    ## verifying all data
    print(line)
    ## writing to angles.csv in Generated folder without spaces
    with open(generated_folder_path+"/"+'output.csv', 'w') as writeFile:
        print("About to write csv")
        writer = csv.writer(writeFile)
        writer.writerow(line)
    ## closing csv file    
    writeFile.close()



    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main(input("time value in seconds:"))
