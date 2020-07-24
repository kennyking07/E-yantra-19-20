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




########################################################################
## using os to generalise Input-Output
########################################################################
codes_folder_path = os.path.abspath('.')
images_folder_path = os.path.abspath(os.path.join('..', 'Images'))
generated_folder_path = os.path.abspath(os.path.join('..', 'Generated'))




############################################
## Build your algorithm in this function
## ip_image: is the array of the input image
## imshow helps you view that you have loaded
## the corresponding image
############################################
def process(ip_image):
    ###########################
    ## Your Code goes here
    ## placeholder image
    img1=cv2.cvtColor(ip_image,cv2.COLOR_BGR2HSV)
    white=[0,0,255]
    a1=cv2.inRange(img1,(0,0,0),(0,0,0))
    s=cv2.moments(a1)
    cx=(s['m10']/s['m00'])
    cy=(s['m01']/s['m00'])
    q1=cv2.inRange(img1,np.array(white),np.array(white))
    y1=cv2.bitwise_and(img1,img1,mask=q1)
    b1=cv2.cvtColor(y1,cv2.COLOR_BGR2GRAY);
    _, c1= cv2.threshold(b1, 50, 255, cv2.THRESH_BINARY)
    cv2.circle(c1,(int(cx),int(cy)),int(50),[0,0,0],-1)
    c2=~c1
    cv2.circle(c2,(int(cx),int(cy)),int(340),[255,255,255],-1)
    c4=~(c1-~(c2))
    sector_image = np.ones(ip_image.shape[:2],np.uint8)*255
    ## check value is white or not
    print(sector_image[0,0])
    ## Your Code goes here
    ###########################
    sector_image=c4
    return sector_image




    
####################################################################
## The main program which provides read in input of one image at a
## time to process function in which you will code your generalized
## output computing code
## Do not modify this code!!!
####################################################################
def main():
    ################################################################
    ## variable declarations
    ################################################################
    i = 1
    ## Reading 1 image at a time from the Images folder
    for image_name in os.listdir(images_folder_path):
        ## verifying name of image
        print(image_name)
        ## reading in image 
        ip_image = cv2.imread(images_folder_path+"/"+image_name)
        ## verifying image has content
        print(ip_image.shape)
        ## passing read in image to process function
        sector_image = process(ip_image)
        ## saving the output in  an image of said name in the Generated folder
        cv2.imwrite(generated_folder_path+"/"+"image_"+str(i)+"_fill_in.png", sector_image)
        i+=1


    

############################################################################################
## main function
############################################################################################
if __name__ == '__main__':
    main()
