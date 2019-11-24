#!/usr/bin/env python
from __future__ import print_function

"""LIST OF FUNCTION (IN ORDER):

color_mask() (not used): keep everything blue in the picture
aruco_detection(): the aruco detection
calcul_distance() : to get the distance once the marker detected
detect_circles() : same logic used in both
detect_ellipse()

check_copper, check_silver, check gold(): 3 filter to keep only one range of color.
image_modification_silver, image_modification_gold, image_modifications : where i put all the functions that transform the image

Callback: where all the principal functions are called
Handler(): the function launched after a "CTRL-C"

main

"""



#too many libaries now

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib as mpl 
import matplotlib.pyplot as plt


def calcul_distance(frame,corners,Size_marker):

    return distance
   
def detect_circles(imgThresholded, frame):

  kernel = np.ones((5,5), np.uint8)
  imgThresholded=cv2.erode(imgThresholded, kernel, iterations=2) # erode the white parts
  imgThresholded = cv2.dilate(imgThresholded, kernel, iterations=1) #dilate the white parts


  cv2.imshow("check",imgThresholded)
  circle_coordinates=[]
  #imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from a 3channel (BGR2HSV) to 1
  circles=cv2.HoughCircles(imgThresholded, cv2.HOUGH_GRADIENT,1, 90,param1=30, param2=30, minRadius=50, maxRadius=100) #Problem if coins are too close or too far

  if circles is not None: #if no circle then don't do it
      circles = np.round(circles[0, :]).astype("int")
      """
      if color_check=="copper":
        coin_check=range(6,8)
      elif color_check=="silver":
        coin_check=range(0,6)
      elif color_check=="gold":
        coin_check=range(0,3)
      """
  number_of_circles=0 #number of circles
  # loop over the (x, y) coordinates and radius of the circles
  try:
    for (x, y, r) in circles: 
      cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
      
      circle_coordinates.append([x,y])  
      print("x and y")
      print(x, y)

      number_of_circles+=1
               
  except TypeError: #in case there is no circles
    pass
  print("number of circles detected")
  print(number_of_circles)
  

  return frame


def detect_ellipse(imgThresholded, frame):

  imgThresholded = cv2.Canny(imgThresholded,80,80)
  _,contours,_= cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)

  """
  if color_check=="copper":
    coin_check=range(6,8)
  elif color_check=="silver":
    coin_check=range(0,6)
  elif color_check=="gold":
    coin_check=range(0,6)
  """
  number_of_circles=0 #number of circles

  for cnt in contours:
    area = cv2.contourArea(cnt) #to check if size MAYBE a coin to begin with (zoom taken into account)

    if area < 900 or len(cnt) < 5 :#or area >10000:  #to get rid of the noises and interference 
      continue
    #(x,y),(MA,ma),angle = cv2.fitEllipse(cnt) #not practical to use
    ellipse = cv2.fitEllipse(cnt)
  
    rbox = cv2.fitEllipse(cnt)
    cv2.ellipse(frame, rbox, (55,255,255), 2,cv2.LINE_AA)

    Ma= ellipse[1][0] #long axe
    ma= ellipse[1][1] #little axe
    r1=Ma/2 
    r2=ma/2
    x=ellipse[0][0]
    y=ellipse[0][1]
    r=(((r1**2+r2**2)/2)**0.5) #we assume we have some kind of circle

    #size_circle= r*2*distance/focallenght
    print("x and y")
    print(x, y)
    print("r")
    print(r)
    print("size")
    number_of_circles+=1

  print("number of ellipses detected")
  print(number_of_circles)
  #cv_txt=cv_image.copy()

  return frame # cv_txt


def detect_triangle(img, frame):
  img = cv2.Canny(img,80,80)
  kernel = np.ones((5,5), np.uint8)
  img=cv2.dilate(img, kernel, iterations=2) # erode the white parts
  img=cv2.erode(img, kernel, iterations=2) #dilate the white parts


  cv2.imshow("show before",img)
  _, contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #chain--> get extremities, no hierarchy
  tri_coordinates = []
  for cnt in contours:
      area = cv2.contourArea(cnt) #to check if size too little to be considered
      #print("area")
      #print(area)
      if area < 5000 or len(cnt) < 20 :#or area >10000:  #to get rid of the noises and interference 
        continue
      [point_x, point_y, width, height] = cv2.boundingRect(cnt)
      approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
      if len(approx) == 3: #if 3 corners
          #(x, y, w, h) = cv2.boundingRect(approx)
          approx = np.array(approx) #to get a real array
          approx=approx.reshape(3,2) #to get rid of the triple []
          
          #print("1")
          #print(approx)
          """ not good, because we would need to be in front of the triangle
          euclidian_dist_one_side=((approx[0][0]-approx[1][0])**2 + (approx[0][1]-approx[1][1])**2)**0.5
          euclidian_dist_side_two=((approx[1][0]-approx[2][0])**2 + (approx[1][1]-approx[2][1])**2)**0.5
          if abs(euclidian_dist_side_two-euclidian_dist_one_side)<40 : #check if two sides of the triangle are equal, taking into account some noise
          """
          tri_coordinates.append([point_x,point_y])
          cv2.drawContours(frame, [cnt], 0, (100, 20, 45), 3)
          print("x,y tri")
          print(point_x,point_y)
          
  print("number of triangle detected")
  print(len(tri_coordinates))
  return frame

def detect_rectangle(img, frame):
  img = cv2.Canny(img,80,80)
  kernel = np.ones((5,5), np.uint8)
  img=cv2.dilate(img, kernel, iterations=3) # erode the white parts
  img=cv2.erode(img, kernel, iterations=3) #dilate the white parts

  cv2.imshow("show before rectangle",img)
  _, contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) #chain--> get extremities, no hierarchy
  square_coordinates, los_coordinates = [],[]
  for cnt in contours:
      area = cv2.contourArea(cnt) #to check if size too little to be considered

      if area < 900 or len(cnt) < 5 :#or area >10000:  #to get rid of the noises and interference 
        continue
          # [point_x, point_y, width, height] = cv2.boundingRect(cnt)
      approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
      if len(approx) == 4: #if 4 corners
          approx = np.array(approx) #to get a real array
          approx=approx.reshape(4,2) #to get rid of the triple []
          (x, y, w, h) = cv2.boundingRect(approx)
          r = w / float(h)  
          # "a square will have an aspect ratio that is approximately
			    # equal to one, otherwise, the shape is a rectangle" https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
          
          if r >= 0.95 and r <= 1.05 :			        
            x_one=abs((approx[0][0]-approx[1][0]))
            x_two=abs((approx[1][0]-approx[2][0]))
            if x_one<10 or x_two<10: #a square has his x in the same axis. (x_one and 2 because we don't know which corner is the first data from)
              square_coordinates.append([x,y])
              cv2.drawContours(frame, [cnt], 0, (255, 0, 45), 3)
              #print("square")
              #print(x,y)
            else:
            #if abs(approx[0][0]-approx[2][1])<50 :
              los_coordinates.append([x,y])
              cv2.drawContours(frame, [cnt], 0, (255, 0, 245), 3)
              #print("losange")
              #print(x,y)  
                
          
  """        
  print("number of rectangle detected")
  print(len(coordinates))
  print(" ")
  #print(coordinates)
  print(" ")
  """
  return frame


def modif_image_fix(imgThresholded):
      
  imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from BGR to GRAY
  imgThresholded=cv2.medianBlur(imgThresholded, 15)
  imgThresholded=cv2.GaussianBlur(imgThresholded, (5,5),0)
  
  _,imgThresholded= cv2.threshold(imgThresholded,127,255,cv2.THRESH_BINARY)

  img = cv2.Canny(imgThresholded,80,80)
  kernel = np.ones((5,5), np.uint8)
  img=cv2.dilate(img, kernel, iterations=3) # erode the white parts
  img=cv2.erode(img, kernel, iterations=3) #dilate the white parts
  cv2.imshow("show result", img)
  return img


def image_modifications(imgThresholded):
  
    imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from BGR to GRAY
    imgThresholded=cv2.medianBlur(imgThresholded, 15)
    imgThresholded=cv2.GaussianBlur(imgThresholded, (5,5),0)
    imgThresholded = cv2.adaptiveThreshold(imgThresholded,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)


    #imgThresholded=cv2.blur(imgThresholded, (5,5))
    #imgThresholded=cv2.GaussianBlur(frame, (15,15),0)
    #ret, imgThresholded = cv2.threshold(imgThresholded, 127, 255, 0)
    #imgThresholded = cv2.adaptiveThreshold(imgThresholded, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 11, 1)


    # Taking a matrix of size 5 or 3as the kernel 
    """
    kernel = np.ones((5,5), np.uint8)
    kernel2 = np.ones((3,3), np.uint8) 
    
  
    # close gaps in between object edges to get perfects circles
    imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=1) #cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) ) 
    imgThresholded=cv2.erode(imgThresholded, kernel, iterations=2) # erode the white parts
    imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=5) #dilate the white parts

    
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=4)

    #imgThresholded = cv2.morphologyEx(imgThresholded, cv2.MORPH_CLOSE, kernel2, iterations=4) #dilation then erosion
    #imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=1)
    #imgThresholded=cv2.erode(imgThresholded, kernel, iterations=1)
    
    cv2.imshow("my_color_is_black",imgThresholded)
    if imgThresholded==[]:
        print(" Error opening image\n")
        return -1;
    #_,contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)
    """

    return imgThresholded
 

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("usb_cam/image_raw/compressed",Image,self.callback)
    #self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber('/image_publisher_1574617647830329591/image_raw',Image,self.callback)#to erase mater on

    #self.image_sub_shape = rospy.Subscriber('/image_publisher_1574617705745817603/image_raw',Image,self.callback2) #to keep, image with shapes
  
  def callback2(self, data2):
    try:
      cv_image_shape = self.bridge.imgmsg_to_cv2(data2, "bgr8")
    except CvBridgeError as e:
      print(e)   
    cv2.imshow("test",cv_image_shape)
    shapes= modif_image_fix(cv_image_shape)
    return shapes
  
  def callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

#parameter of my computer's camera used to print the aruco axes
    #cameraMatrix =np.array([[611.029350, 0.000000, 291.500762],[0.000000, 608.520033, 236.523594],[0.000000, 0.000000, 1.000000]])
    #distCoeffs =np.array([[-0.028521,-0.110481, 0.001912, -0.009806, 0.000000]])
    frame=cv_image.copy()#in case no ids detected
    imgThresholded=image_modifications(frame)
    cv2.imshow('show without circles',imgThresholded)
    ori_cir=detect_circles(imgThresholded,frame)
    #ori_ell=detect_ellipse(imgThresholded,frame)#not good, circle is better
    #ori_tri=detect_triangle(imgThresholded,frame)
    #ori_rect=detect_rectangle(imgThresholded, frame)

    cv2.imshow("circles",ori_cir)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  
  

def main(args):
 
  ic = image_converter()

  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except (KeyboardInterrupt, SystemExit):

    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
