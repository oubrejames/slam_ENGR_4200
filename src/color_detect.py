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
		mask1 = cv2.inRange(img_hsv, (0, 50, 50), (10, 255, 255))
		mask2 = cv2.inRange(img_hsv, (170, 50, 20), (180, 255, 255))

		## Merge the mask and crop the red regions
		mask = cv2.bitwise_or(mask1, mask2)

		img_blr = cv2.blur(image,(10,10))

		just_red = cv2.bitwise_and(img_blr, img_blr, mask=mask)

		red_blr = cv2.blur(just_red, (10,10))
		just_red2 = cv2.bitwise_and(red_blr, red_blr, mask=mask)

		ret, thresh1 = cv2.threshold(just_red2, 0, 255, cv2.THRESH_BINARY)

		#Flag if red is seen
		#cv2.imshow("images", just_red)
		#cv2.imshow("images1", just_red2)
		#cv2.imshow("thresh1", thresh1)

		#cv2.waitKey(0)

		red_detected = False
		i = 0
		j = 0
		#If i see red set flag
		while i < image.shape[0]:
			i += 10
			while j < image.shape[1]:
				j += 10
				if thresh1.any() > 0:
					red_detected = True
					print(red_detected)
					break
			if red_detected:
				break

		print(red_detected)
		return red_detected




if __name__ == '__main__' :
	st = img_proc()
	imag = cv2.imread("FOR_SURE_NO_RED.PNG")
	#imag = cv2.imread("red_stuff.PNG")
	st.color_detect(imag)


