import cv2
import numpy as np
import random
import matplotlib.pyplot as plt
import math

print(cv2.__version__)
#print cv2.getBuildInformation()

LOWER_GREEN_HUE = 50 # gimp: 100
LOWER_GREEN_SAT = 60 # gimp: 23.43
LOWER_GREEN_VAL = 60 # gimp: 23.43

UPPER_GREEN_HUE = 80 # gimp: 300
UPPER_GREEN_SAT = 255 # gimp: 100
UPPER_GREEN_VAL = 255 # gimp: 100
MIN_AREA = 4000
#CAMERA_ANGLE = 90 #degrees, INCORRECT VALUE
FOCAL_LENGTH = 1170.0 #2017 value
CLOSEST_DIST = 8 # inches
#CAM_HEIGHT_OFFSET = 3#feet, INCORRECT VALUE
#BOUNDING_REAL_HEIGHT = 5.7/12 #feet


PIXEL_WIDTH = 1280
PIXEL_HEIGHT = 720
CENTER_X = 640
CENTER_Y = 360

lower_green = np.array([LOWER_GREEN_HUE, LOWER_GREEN_SAT, LOWER_GREEN_VAL])
upper_green = np.array([UPPER_GREEN_HUE, UPPER_GREEN_SAT, UPPER_GREEN_VAL])

ANGLE_MIN_LEFT = None
ANGLE_MAX_LEFT = None
ANGLE_MIN_RIGHT = None
ANGLE_MAX_RIGHT = None

ASPECT_RATIO_MIN = 0.3636 - 0.3
ASPECT_RATIO_MAX = 0.3636 + 0.3

class Boxes():
  def __init__(self, boxes, angle):
    self.boxes = boxes
    self.angle = angle

#def FilterAngle(angle):

rotated_boxes = []
box_centers = []

#cap = cv2.VideoCapture(0)
#change id above

'''
while True:
  ret,img = cap.read() #returns frame

  if ret is False:
    print("hello, no image, trying again")
    continue
   ''' 
img = cv2.imread('hatch_5ft_3.jpg')

orig = img.copy()
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
green_range = cv2.inRange(hsv_img, lower_green, upper_green)
green_range = cv2.GaussianBlur(green_range, (9,9),2)

contours, hierarchy = cv2.findContours(green_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#print(contours)
color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
#cv2.drawContours(orig,contours, -1, color, 3)
contours_poly = []
bound_rect = []

                                                                                                                                                                                                                                                                                       
left_contour = None
right_contour = None

print("before contour loop")
for i in range(len(contours)):
  print ("loop")
  #print(i)
  #if cv2.waitKey(0):
  #  break
  #print (contours[i])
  if cv2.contourArea(contours[i]) < MIN_AREA:
    print("hello")
    continue
  color = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
  contours_poly.append(cv2.approxPolyDP(contours[i], 3, True))
  x,y,w,h = cv2.boundingRect(contours[i])
  bound_rect.append((x,y,w,h))
  cv2.rectangle(orig, (x,y),   (x+w,y+h), color, 2)
  rect = cv2.minAreaRect(contours[i])
  rect_center = rect[0]
  rect_dim = rect[1]
  rect_angle = rect[2]
  print (rect)
  if rect_angle < -45:
    aspectRatio = rect_dim[1]/rect_dim[0]
  elif rect_angle > -45:
    aspectRatio = rect_dim[0]/rect_dim[1]
  else:
    print("not rectangle")
    aspectRatio = 0
  print("before aspect ratio")
  if ASPECT_RATIO_MIN < aspectRatio and aspectRatio < ASPECT_RATIO_MAX:
    #print("yo")
    print(i)
    box = cv2.boxPoints(rect)
    targetBox = Boxes(box, rect_angle)
    rotated_boxes.append(targetBox)
    #print box
    box = np.int0(box)
    print("draw box")
    cv2.drawContours(orig, [contours[i]], 0, color, 2)
    cv2.drawContours(orig, [box],0,color,2)
  else: 
    print ("huh")
    continue
    
'''
#check number of rotated boxes
if len(rotated_boxes) < 2:
cv2.imshow('orig', orig)
elif len(rotated_boxes) = 2:
  #calculate values for the pair
else:
  #more than 2, find correct pair
'''

print ("whatup dude")

'''
print(len(rotated_boxes))
for i in range(len(rotated_boxes)):
  print (i)
  box1 = rotated_boxes[i] #left box
  #box2 = rotated_boxes[i] #rightbox
  boxCoord = box1.boxes
  # topDiff = 
  print("box")
  print(box1.angle)
  print(boxCoord)
  '''

print(len(rotated_boxes))

for i in range(len(rotated_boxes)):
 #if ANGLE_MIN_LEFT < angle < ANGLE_MIN_RIGHT:

 print (rotated_boxes[i].angle)
 #print rotated_boxes[i]
 print("box")


left_cnt = None
right_cnt = None

#print bound_rect[0]
#print bound_rect[1]
if len(rotated_boxes) < 2:
  print ("bro, there aren't enough rotated boxes, please try again")
  cv2.imshow('orig', orig)
  '''
  plt.figure(0)
  plt.imshow(orig, cmap = 'gray', interpolation = 'bicubic')
  plt.xticks([]), plt.yticks([])'''

  #plt.figure(1)
  #plt.imshow(green_range, cmap='gray', interpolation='bicubic')
  cv2.imshow('green_range', green_range)
  #plt.xticks([]), plt.yticks([])
  #plt.show()
  #continue
elif bound_rect[0][0] < bound_rect[1][0]:
  print ("lr1")
  left_cnt = 0
  right_cnt = 1
else:
  print("lr2")
  left_cnt = 1
  right_cnt = 0

print("what is going on")

if len(rotated_boxes) == 2:
  print ("this is a okay")
  

if len(bound_rect) >= 2:
  left_rightedge = bound_rect[left_cnt][0] + bound_rect[left_cnt][2]
  right_leftedge = bound_rect[right_cnt][0]
  center_of_target = (left_rightedge + right_leftedge)/2.0
  angle_to_center = math.atan((PIXEL_WIDTH/2.0 - center_of_target)/FOCAL_LENGTH) * 180.0/math.pi

  #print(angle_to_center)

  pixel_closest_dist = right_leftedge - left_rightedge
  #print pixel_closest_dist

  distance = CLOSEST_DIST * FOCAL_LENGTH / pixel_closest_dist
  #print(distance)

#cv2.imshow('orig', orig)
'''
plt.figure(0)
plt.imshow(orig, cmap = 'gray', interpolation = 'bicubic')
plt.xticks([]), plt.yticks([]) '''

#plt.figure(1)
#plt.imshow(green_range, cmap='gray', interpolation='bicubic')
cv2.imshow('orig', orig)
cv2.imshow('green_range', green_range)
cv2.waitKey(0)
cv2.destroyAllWindows()
#plt.xticks([]), plt.yticks([])
#plt.show()

#cv2.destroyAllWindows




'''
angle = None #from center of screen,(+) if right
distance = None

if len(bound_rect) >=2:
  x1, y1, w1, h1 = bound_rect[0]
  x2, y2, w2, h2 = bound_rect[1]
  centerX = (x1+x2)/2
  xOffset = centerX-img.cols
  angle = xOffset*CAMERA_ANGLE/img.cols

  #divide by average bounding height
  camDistance = BOUNDING_REAL_HEIGHT*FOCAL_LENGTH/((h1+h2)/2)
  robotDistance = math.sqrt(-CAM_HEIGHT_OFFSET*CAM_HEIGHT_OFFSET+camDistance*camDistance)

  print("Angle: "+angle+" Camera Distance: "+camDistance+" robot distance: "+robotDistance)
else:
  print("Two bounding boxes not detected. no angle")

'''



#cv2.imshow('original and contours', orig)
#cv2.imshow('green stuff lol', green_range)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
