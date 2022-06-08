import cv2 as cv

import numpy as np

kernel = np.ones((5,5), np.uint8)

red_hexagons_classifier = cv.CascadeClassifier("./cascadeRedHexagon.xml")


originalFrame = cv.imread("stop.png")
originalFrame = cv.rotate(originalFrame,cv.ROTATE_180)
#frame = cv.resize(originalFrame,(400,240), interpolation = cv.INTER_AREA)
frame = originalFrame
haarDetection = frame.copy()
HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)     
	
red1 = cv.inRange(HSV,(145,70,50),(180,255,255))
red2 = cv.inRange(HSV,(0,70,50),(10,255,255))
red = cv.bitwise_or(red1,red2)	
print(red.shape)
mask1 = np.zeros((red.shape[0]+80,red.shape[1]))
mask1[80:,:] = red
red = mask1


	
redHexagons = red_hexagons_classifier.detectMultiScale(red, scaleFactor=1.2, minNeighbors=150, minSize=(40,40))

	
for (x,y,w,h) in redHexagons:
	cv.rectangle(haarDetection, (x,y),(x+w,y+h),(0,0,255),2)
	cv.putText(haarDetection,'redHexagon',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)

	"""
	red_hexagon_roi = frameCopy[y:y+h,x:x+w]
	red_hexagon_roi = cv.resize(red_hexagon_roi,(45,45), interpolation = cv.INTER_AREA)   
	red_hexagon_roi = preprocessing(red_hexagon_roi)
	red_hexagon_roi = red_hexagon_roi.reshape(1, 45, 45, 1)

	cv.rectangle(frameCopy, (x,y),(x+w,y+h),(0,0,255),2)
	cv.putText(frameCopy,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
	"""

cv.imshow('haarDetection',haarDetection)
cv.imshow('Original Frame',originalFrame)

cv.imshow('Red Image',red)

cv.waitKey(0)