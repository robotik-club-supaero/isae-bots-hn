#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import time


# dictionary to attribute the detected marker id to a type of sample
markerAttributionDict = {47 : (0,0,255),  # red sample
						 13 : (255,0,0),  # blue sample
						 36 : (0,255,0),  # green sample
						 17 : (0,67,162),  # upside down sample
						 42 : (128,128,128)  # marker at the center of the table
						 }

def main():

	# load image (photo taken by the camera)
	cap = cv2.VideoCapture(0)
	while True:
		ret, image = cap.read()
		image = cv2.resize(image, (1440, 810))
	# define aruco dictionary
		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)  # TODO : optim avec seulement les bons ids de marqueurs ?

	# define aruco parameters (leave defaults)
		arucoParams = cv2.aruco.DetectorParameters_create()

	# detect the aruco markers
		begin = time.time()
		(corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
		print("Time taken to detect markers : {} s".format(round(time.time()-begin, 4)))

		nbDetected = len(corners)

	# display the corners of the markers depedning of their color
		for k in range(nbDetected):

			cornerList = corners[k][0]
			markerId = ids[k][0]

			for i in range(4):
				image = cv2.drawMarker(image, (int(cornerList[i][0]),int(cornerList[i][1])), color=markerAttributionDict[markerId], markerType=cv2.MARKER_CROSS, thickness=2)


	# display image and wait for a key to be pressed
		#cv2.imshow('Aruco Test', image)
		cv2.imwrite("saved_image.jpeg", image)
		cv2.waitKey(0)
		break



if __name__ == '__main__':
	main()
 