import cv2
import numpy as np
import os

def partA():
    label=[]
    ext=[]
    path='../Images/'
    img_arr=[]
    for img_name in os.listdir(path):
      if img_name.endswith(".jpg"):
         label.append(img_name.split('.')[0])
         ext.append(os.path.splitext(img_name)[1])
         img_arr.append(os.path.join(path,img_name))
    fin = [] 
    for i in range(0, len(label)): 
       fin.append((label[i] + ext[i]))
    x=[]; u=[]; o=[]; p=[]; v=[]; m=[]       
    for imagePath in img_arr:
       image = cv2.imread(imagePath)
       h,w,c=image.shape
       arr=np.array(image);
       h1=(int)(h/2)
       w1=(int)(w/2)
       r,g,b=arr[h1,w1]
       x.append(h)
       u.append(w)
       o.append(c)
       p.append(r)
       v.append(g)
       m.append(b)

    row1=str(fin[0]),str(x[0]),str(u[0]),str(o[0]),str(p[0]),str(v[0]),str(m[0])
    row2=str(fin[1]),str(x[1]),str(u[1]),str(o[1]),str(p[1]),str(v[1]),str(m[1])
    row3=str(fin[2]),str(x[2]),str(u[2]),str(o[2]),str(p[2]),str(v[2]),str(m[2])
    row4=str(fin[3]),str(x[3]),str(u[3]),str(o[3]),str(p[3]),str(v[3]),str(m[3])
    f= open('../Generated/stats.csv','w+')
    rowa1=','.join(row1);rowa2=','.join(row2);rowa3=','.join(row3);rowa4=','.join(row4);
    f.write("%s \n " %rowa1);f.write("%s \n " %rowa2);f.write("%s \n " %rowa3);f.write("%s \n\r " %rowa4)

def partB():
    img=cv2.imread('../Images/cat.jpg');
    c=img.copy()
    c[:,:,1]=0;
    c[:,:,0]=0;
    cv2.imwrite('../Generated/cat_red.jpg',c);

def partC():
    img = cv2.imread('../Images/flowers.jpg')
    b,g,r=cv2.split(img)
    alpha=np.ones(b.shape,dtype=b.dtype)*127;
    x=np.dstack([img ,alpha])
    cv2.imwrite('../Generated/flowers_alpha.png',x)
    
def partD():
    img=cv2.imread('../Images/horse.jpg');
    grey = lambda rgb : np.dot(rgb[... , :3] , [0.3 , 0.59, 0.11]) 
    grey = grey(img)
    cv2.imwrite('../Generated/horse_gray.jpg',grey)

partA()
partB()
partC()
partD()
