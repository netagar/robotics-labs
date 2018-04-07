#!/usr/bin/env python3

import cv2
import sys
import copy

import numpy as np

try:
	from PIL import Image, ImageDraw, ImageFont
except ImportError:
	sys.exit('install Pillow to run this code')


def find_ball(opencv_image, debug=False):
	"""Find the ball in an image.
		
		Arguments:
		opencv_image -- the image
		debug -- an optional argument which can be used to control whether
				debugging information is displayed.
		
		Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
	"""

	edge_threshold = 100
	accumulator_threshold = 35

	blurred = cv2.GaussianBlur(opencv_image, (9, 9), 2)

	if debug:
		edges = cv2.Canny(blurred, edge_threshold, edge_threshold/2)
		pil_image = Image.fromarray(edges)
		pil_image.show() 

	circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 
		dp=1, minDist=30,
		param1=edge_threshold, param2=accumulator_threshold,
		minRadius=10, maxRadius=100)
	
	if circles is None:
		return [0, 0, 0]

	# Reduce the dimensionality
	circles = circles[0,:]

	print(len(circles))
	if debug:
		#print(circles)
		display_circles(blurred, circles)
		input("Press Enter to continue...")

	if len(circles) > 0:
		return circles[0]
	else:
		return [0, 0, 0]


def display_circles(opencv_image, circles, best=None):
	"""Display a copy of the image with superimposed circles.
		
	   Provided for debugging purposes, feel free to edit as needed.
	   
	   Arguments:
		opencv_image -- the image
		circles -- list of circles, each specified as [x,y,radius]
		best -- an optional argument which may specify a single circle that will
				be drawn in a different color.  Meant to be used to help show which
				circle is ranked as best if there are multiple candidates.
		
	"""
	#make a copy of the image to draw on
	circle_image = copy.deepcopy(opencv_image)
	circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
	
	for c in circles:
		# draw the outer circle
		cv2.circle(circle_image,(c[0],c[1]),c[2],(255,255,0),2)
		# draw the center of the circle
		cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),3) 
		# write coords
		cv2.putText(circle_image,str(c),(c[0],c[1]),cv2.FONT_HERSHEY_SIMPLEX,
					.5,(255,255,255),2,cv2.LINE_AA)            
	
	#highlight the best circle in a different color
	if best is not None:
		# draw the outer circle
		cv2.circle(circle_image,(best[0],best[1]),best[2],(0,0,255),2)
		# draw the center of the circle
		cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
		# write coords
		cv2.putText(circle_image,str(best),(best[0],best[1]),cv2.FONT_HERSHEY_SIMPLEX,
					.5,(255,255,255),2,cv2.LINE_AA)            
		
	
	#display the image
	pil_image = Image.fromarray(circle_image)
	pil_image.show()    
	  
if __name__ == "__main__":
	pass
