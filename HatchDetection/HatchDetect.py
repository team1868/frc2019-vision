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


ASPECT_RATIO_MIN = 0.3636 - 0.3
ASPECT_RATIO_MAX = 0.3636 + 0.3

class Boxes():
  def __init__(self, boxes, angle, type):
    self.boxes = boxes
    self.angle = angle
    self.type = type

#def FilterAngle(angle):
rotated_boxes = []
box_centers = []
pairs = []

#cap = cv2.VideoCapture(0)
#change id above
'''
while True:
  ret,img = cap.read() #returns frame

  if ret is False:
    print("hello, no image, trying again")
    continue
   ''' 
img = cv2.imread('edited164751.jpg')

orig = img.copy()
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
green_range = cv2.inRange(hsv_img, lower_green, upper_green)
green_range = cv2.GaussianBlur(green_range, (9,9),2)

contours, hierarchy = cv2.findContours(green_range, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#img for serena?
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
    #print("hello")
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
  type = 0
  print (rect)
  if (rect_angle <= -45):
    #left rotated boxes
    type = 1
    aspectRatio = rect_dim[1]/rect_dim[0]
  elif (-45 < rect_angle):
     #right rotated boxes
     type = 2
     aspectRatio = rect_dim[0]/rect_dim[1]
  else:
    print("not rectangle")
    aspectRatio = 0
  print("before aspect ratio")
  if ASPECT_RATIO_MIN < aspectRatio and aspectRatio < ASPECT_RATIO_MAX:
    #print("yo")
    print(i)
    box = cv2.boxPoints(rect)
    targetBox = Boxes(box, rect_angle, type)
    rotated_boxes.append(targetBox)
    #print box
    box = np.int0(box)
    print("draw box")
    cv2.drawContours(orig, [contours[i]], 0, color, 2)
    cv2.drawContours(orig, [box],0,color,2)
  else: 
    print ("huh, where the box at?")
    continue
    

print(len(rotated_boxes))
for i in range(len(rotated_boxes)):
  print (i)
  box1 = rotated_boxes[i]
  boxCoord = box1.boxes 
  print("box")
  print(box1.angle)
  print(boxCoord)


print(len(rotated_boxes))

for i in range(len(rotated_boxes)):
 print(rotated_boxes[i].angle)
 print("box")


left_cnt = None
right_cnt = None

if len(rotated_boxes) < 2:
  print("bro, there aren't enough rotated boxes, please try again")
  cv2.imshow('orig', orig)
  cv2.imshow('green_range', green_range)
  # TODO: maybe make it back up and run through again hatchdetect again
  '''
  plt.figure(0)
  plt.imshow(orig, cmap = 'gray', interpolation = 'bicubic')
  plt.xticks([]), plt.yticks([])

  plt.figure(1)
  plt.imshow(green_range, cmap='gray', interpolation='bicubic')
  plt.xticks([]), plt.yticks([])
  plt.show()
  continue
  '''
temporary = 0.00
distance_from_center = 15000.0
center_of_target = 0.00
if len(rotated_boxes) == 2:
  print("got 2")
  print(rotated_boxes[0].boxes[0][0])
  print(rotated_boxes[1].boxes[0][0])
  if (rotated_boxes[1].boxes[0][0] < rotated_boxes[0].boxes[0][0]) and (rotated_boxes[0].angle > rotated_boxes[1].angle):
    #read right to left
    print("'tis a pair'")
    temporary = (rotated_boxes[0].boxes[1][0] + rotated_boxes[1].boxes[3][0])/2
    if (abs(distance_from_center) > abs(640 - temporary)):
      distance_from_center = temporary - 640
  elif (rotated_boxes[1].boxes[0][0] > rotated_boxes[0].boxes[0][0]) and (rotated_boxes[0].angle < rotated_boxes[1].angle):
    #read left to right
    print("'tis a pair")
    temporary = (rotated_boxes[0].boxes[3][0] + rotated_boxes[1].boxes[1][0])/2
    if (abs(distance_from_center) > abs(640 - temporary)):
      distance_from_center = temporary - 640
  else:
    print("sorry, it ain't a pair")
    # TODO: maybe make it back up and run through again hatchdetect again

which_box = 0
if len(rotated_boxes) > 2:
  print("excess boxes brotha")
  print(rotated_boxes[0].boxes[0][0])
  for i in range(len(rotated_boxes)-1):
    print(rotated_boxes[i+1].boxes[0][0])
    if (rotated_boxes[i+1].boxes[0][0] < rotated_boxes[i].boxes[0][0]) and (rotated_boxes[i].angle > rotated_boxes[i+1].angle):
      #read right to left
      temporary = (rotated_boxes[i].boxes[1][0] + rotated_boxes[i+1].boxes[3][0])/2
      #print("yo")
      print(temporary)
      if (abs(distance_from_center) > abs(640 - temporary)):
        #print("stop")
        distance_from_center = temporary - 640
        print(distance_from_center)
        which_box = i
      print("yes")
    elif (rotated_boxes[i+1].boxes[0][0] > rotated_boxes[i].boxes[0][0]) and (rotated_boxes[i].angle < rotated_boxes[i+1].angle):
      #read left to right
      temporary = (rotated_boxes[i].boxes[3][0] + rotated_boxes[i+1].boxes[1][0])/2
      #print("yo")
      print(temporary)
      if (abs(distance_from_center) > abs(640 - temporary)):
        #print("stop")
        distance_from_center = temporary - 640
        print(distance_from_center)
        which_box = i
      print("yes")
    else:
      print("nah man")
#print(rotated_boxes[which_box].boxes[0][0])
#print(rotated_boxes[which_box + 1].boxes[0][0])

center_of_target = distance_from_center + 640
print(center_of_target)
angle_to_center = (math.atan((PIXEL_WIDTH/2.0 - center_of_target)/FOCAL_LENGTH) * 180.0/math.pi)*-1
print(angle_to_center)

      #change above so works for four and correct angle orders
      #pairs[0][0]=[i][i+1]

  

'''
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
'''
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
