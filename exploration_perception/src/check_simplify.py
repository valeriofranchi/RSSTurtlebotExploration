#! /usr/bin/env python

import cv2
import numpy as np

def simplify(image):
    #load image
	image = cv2.imread(image) 
	#resize image 
	ratio = 500 / float(image.shape[0])
	resized = cv2.resize(image, (int(ratio * image.shape[1]), 500))
	cutoff_point = resized.shape[0] // 8
	cutoff = np.zeros((cutoff_point, resized.shape[1], 3), np.uint8)
	resized[:cutoff_point, :] = cutoff
	#convert to hsv 
	hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
	h, s, v = cv2.split(hsv)
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
	s = clahe.apply(s)
	v = clahe.apply(v)
	hsv = cv2.merge([h, s, v])
	#create bounds, apply inRange function 
	cardboard_lower, cardboard_upper = (0, 92, 0), (23, 255, 159)
	cardboard_mask = cv2.inRange(hsv, cardboard_lower, cardboard_upper) 
	cv2.imshow("hsv thresh", cardboard_mask)
	#apply opening operation to remove blobs and bitwise AND it with resized image 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	opened = cv2.morphologyEx(cardboard_mask, cv2.MORPH_OPEN, kernel, iterations=3)
	cv2.imshow("morph ops", opened)
	output = cv2.bitwise_and(resized, resized, mask=opened)
	cv2.imshow("bit AND", output)
	
	#find non zero pixels and then positions in matrix of these pixels 
	non_zero_y = np.nonzero(output)[0]
	#find max y value of these positions 
	if len(non_zero_y) > 0:
		maxy = np.min(non_zero_y)
		
		#convert to all values above maxy to black in image 
		#for resized one (for displaying and debugging purposes)
		#b = np.tile(resized[maxy, :, 0], (maxy, 1))
		#g = np.tile(resized[maxy, :, 1], (maxy, 1))
		#r = np.tile(resized[maxy, :, 2], (maxy, 1))
		#zeros = np.zeros((image[:maxy, :].shape), np.uint8)
		#zeros.fill((15, 150, 80))
		#overlay = cv2.merge([b, g, r])
		resized[:maxy, :, :] = (39, 63, 83)
		cv2.imshow("final", resized)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		
		#convert maxy for original size image and make every pixel above it black
		maxy_converted = int((maxy / float(resized.shape[0])) * image.shape[0]) 
		if maxy_converted - 30 > 0:
			overlay = np.array(image[maxy_converted-30, :] * maxy_converted-30)
			#zeros = np.zeros((image[:maxy_converted, :].shape), np.uint8)
			b = np.tile(image[maxy_converted-30, :, 0], (maxy_converted-30, 1))
			g = np.tile(image[maxy_converted-30, :, 1], (maxy_converted-30, 1))
			r = np.tile(image[maxy_converted-5, :, 2], (maxy_converted-30, 1))
			overlay = cv2.merge([b, g, r])
			image[:maxy_converted - 30, :] = overlay
			#image[:maxy_converted, :] = zeros"""
	return image

simplify("/home/valeriofranchi/catkin_ws/src/exploration_perception/test/frame0001.jpg")

