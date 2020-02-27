#!/usr/bin/env python
import roslib
roslib.load_manifest('car_hough')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

pub=rospy.Publisher('Hough_Info_Derecha', String, queue_size=1)
pub2=rospy.Publisher('Hough_Info_Izquierda', String, queue_size=1)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ####division
    #top
    height,width=cv_image.shape[:2]
    start_row, start_col=int(0),int(0)
    end_row,end_col=int(height*.8),int(width)
    cropped_top=cv_image[start_row:end_row,start_col:end_col]
    #cv2.imshow("Cropped Top",cropped_top)
    #down 1
    cropped_down1=cv_image[end_row:height,start_col:int(end_col*.33)]
    #down 2
    cropped_down2=cv_image[end_row:height,int(end_col*.33):int(end_col*.66)]
    #down 3
    cropped_down3=cv_image[end_row:height,int(end_col*.66):int(end_col*.99)]

    #####procesamiento
    try:
        #procesa ventana 1
        grayD1=cv2.cvtColor(cropped_down1,cv2.COLOR_BGR2GRAY)
        edgesD1=cv2.Canny(grayD1,75,150)
        cv2.imshow("Canny",edgesD1)
        linesD1=cv2.HoughLinesP(edgesD1,1,np.pi/180,30,maxLineGap=250)
        datoD1=linesD1[0]
        d1X1,d1Y1,d1X2,d1Y2=datoD1[0]
        cv2.line(cropped_down1,(d1X1,d1Y1),(d1X2,d1Y2),(0,255,0),3)
        pub.publish("vetana 1")
    except Exception as e:
        print "no hay linea en ventana 1"
        try:
            #si no hay en ventana 1 hay que buscar en ventana 2
            grayD2=cv2.cvtColor(cropped_down2,cv2.COLOR_BGR2GRAY)
            edgesD2=cv2.Canny(grayD2,75,150)
            linesD2=cv2.HoughLinesP(edgesD2,1,np.pi/180,30,maxLineGap=250)
            datoD2=linesD2[0]
            d2X1,d2Y1,d2X2,d2Y2=datoD2[0]
            cv2.line(cropped_down2,(d2X1,d2Y1),(d2X2,d2Y2),(0,255,0),3)
            pub.publish("ventana 2")
        except Exception as e:
            print "no hay linea en ventana 2"

    try:
        #proceso ventana 3
        grayD3=cv2.cvtColor(cropped_down3,cv2.COLOR_BGR2GRAY)
        edgesD3=cv2.Canny(grayD3,75,150)
        linesD3=cv2.HoughLinesP(edgesD3,1,np.pi/180,30,maxLineGap=250)
        datoD3=linesD3[0]
        d3X1,d3Y1,d3X2,d3Y2=datoD3[0]
        cv2.line(cropped_down3,(d3X1,d3Y1),(d3X2,d3Y2),(0,255,0),3)
        pub2.publish("ventana 3")
    except Exception as e2:
        print "no hay nada en ventana 3"
        try:
            #si no hay en ventana 3 hay que buscar en ventana 2
            grayD2=cv2.cvtColor(cropped_down2,cv2.COLOR_BGR2GRAY)
            edgesD2=cv2.Canny(grayD2,75,150)
            linesD2=cv2.HoughLinesP(edgesD2,1,np.pi/180,30,maxLineGap=250)
            datoD2=linesD2[0]
            d2X1,d2Y1,d2X2,d2Y2=datoD2[0]
            cv2.line(cropped_down2,(d2X1,d2Y1),(d2X2,d2Y2),(0,255,0),3)
            pub2.publish("ventana 2")
        except Exception as e:
            print "no hay nada en ventana 2"

    #todo####
    cv2.line(cv_image,(0,int(height*.5)),(width,int(height*.5)),(0,0,255),3)
    cv2.line(cv_image,(int(width*.33),int(height*.5)),(int(width*.33),height),(0,0,255),3)
    cv2.line(cv_image,(int(width*.66),int(height*.5)),(int(width*.66),height),(0,0,255),3)
    cv2.line(cv_image,(0,int(height*.5)),(end_col,int(height*.5)),(0,0,255),3)
    cv2.imshow("Complete",cv_image)

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
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
