# import the necessary packages
import numpy as np
import argparse
import cv2

#TODO: Make it a stream from the robots camera
image = cv2.imread("red_stuff.PNG")


# define the list of boundaries
boundaries = [([20, 20, 50], [250, 250, 255])]

# loop over the boundaries
for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")
	# find the colors within the specified boundaries and apply
	# the mask
	mask = cv2.inRange(image, lower, upper)
	output = cv2.bitwise_and(image, image, mask = mask)
	# show the images
	cv2.imshow("images", output)

	#TODO: If red detected, save its position and mark on map
	cv2.waitKey(0)

