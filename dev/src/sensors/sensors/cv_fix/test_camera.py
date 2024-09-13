#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

def main():

	cap = cv2.VideoCapture(0)
	while True:
		ret, image = cap.read()
		image = cv2.resize(image, (1440, 810))
		cv2.imwrite("saved_image.jpeg", image)
		cv2.waitKey(10)
		break

if __name__ == '__main__':
	main()
