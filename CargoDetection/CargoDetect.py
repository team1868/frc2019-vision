#note to Kaylia: run from command prompt
#using python3

import numpy as np
import cv2

print("openCV version: "+cv2.__version__)
cam = cv2.VideoCapture(0)

lower_orange = np.array([-10, 150, 110])#([0, 150, 50])#([0, 200, 50]) #hue, saturation, brightness
upper_orange = np.array([90, 255, 255])#([50, 255, 255])#([60, 255, 255])
while True:
    show, img = cam.read()
    if show:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        smoothMask = cv2.bilateralFilter(mask,15,75,75)
        coloredIn = cv2.bitwise_and(img,img, mask= smoothMask)
        
        im2, cc, hierarchy = cv2.findContours(smoothMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(cc) > 0:
            largestIndex = 0
            for i in range(len(cc)):
                area = cv2.contourArea(cc[i])
                if area > cv2.contourArea(cc[largestIndex]):
                    largestIndex = i

            if(cv2.contourArea(cc[largestIndex]) >= 800):
                (x,y),radius = cv2.minEnclosingCircle(cc[largestIndex])
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(coloredIn,center,radius,(0,255,0),2) #bgr       

        #cv2.imshow('Cargo Detection',img)
        #cv2.imshow('mask',mask)
        #cv2.imshow('smooth mask',smoothMask)
        cv2.imshow('coloredIn',coloredIn)
    if cv2.waitKey(1) == 27: #esc key to quit program
        break

cam.release()
cv2.destroyAllWindows();
