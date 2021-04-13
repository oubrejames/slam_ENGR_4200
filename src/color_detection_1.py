# import the necessary packages
import numpy as np
import argparse
import cv2


class img_proc(object):

	def __init__(self):
		self.red_x = []
		self.red_y = []


	def color_detect(self, image):

		img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		## Gen lower mask (0-5) and upper mask (175-180) of RED
		mask1 = cv2.inRange(img_hsv, (0, 50, 20), (5, 255, 255))
		mask2 = cv2.inRange(img_hsv, (175, 50, 20), (180, 255, 255))

		## Merge the mask and crop the red regions
		mask = cv2.bitwise_or(mask1, mask2)
		just_red = cv2.bitwise_and(image, image, mask=mask)
		#ret, thresh1 = cv2.threshold(just_red, 127, 255, cv2.THRESH_BINARY)

		#Flag if red is seen
		cv2.imshow("images", just_red)
		#cv2.imshow("images", thresh1)

		cv2.waitKey(0)

		red_detected = False
		#If i see red set flag
		for i in range(0, image.shape[0]):
			for j in range(0, image.shape[1]):
				if just_red.any() > 0:
					red_detected = True
					break

		return red_detected


