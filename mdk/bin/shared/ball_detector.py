import numpy as np 
import cv2

import marker as mk

ball_radius = 0

def readImage( filename ):
	img = cv2.imread(filename, 1)
	return img

def getWeight( image, res, f ):
	h,w,d = image.shape

	color = res['color']
	pos = res['position']
	
	x = float(pos[0])/w
	weight = f(x)

	"""
	print 'Getting weights: '
	print color
	print pos
	print x
	print weight
	"""

	return weight


def getPositionWeights( leftImage, rightImage ):
	# L, R, C
	ballListLeft = detectBall( leftImage )
	ballListRight = detectBall( rightImage )


	leftWeights = {}
	leftCenterWeights = {}
	rightWeights = {}
	RightCenterWeights = {}

	sigma = 0.5
	fl = lambda x : np.exp( -(x)**2/(2*sigma**2) )
	fr = lambda x : np.exp( -(x - 1.0)**2/(2*sigma**2))

	for res in ballListLeft:
		leftWeights[res['color']] = getWeight( leftImage, res, fl )
		leftCenterWeights[res['color']] = getWeight( leftImage, res, fr )


	for res in ballListRight:
		rightWeights[res['color']] = getWeight( rightImage, res, fr )
		RightCenterWeights[res['color']] = getWeight( rightImage, res, fl )
   
	lb = leftWeights['blue'] if 'blue' in leftWeights else 0.0
	lr = leftWeights['red'] if 'red' in leftWeights else 0.0
	cba = leftCenterWeights['blue'] if 'blue' in leftCenterWeights else 0.0
	cbb = RightCenterWeights['blue'] if 'blue' in RightCenterWeights else 0.0
	cb = (cba + cbb)/2.0 
	cra = leftCenterWeights['red'] if 'red' in leftCenterWeights else 0.0
	crb = RightCenterWeights['red'] if 'red' in RightCenterWeights else 0.0
	cr = (cra + crb)/2.0
	rr = rightWeights['red'] if 'red' in rightWeights else 0.0
	rb = rightWeights['blue'] if 'blue' in rightWeights else 0.0

	
	weights = {'blue': [lb, cb, rb], 'red': [lr, cr, rr]}
	return weights

def detectBall( image ):
	# is blue or red
	# distance
	# res = {'color': 'blue'; 'radius': 2.3, 'position': [a,b]}	
	ball_list = []
	#image = cv2.resize(image, (640, 480))    

	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

#	cv2.imshow('image', image)

	redMask = mk.get_mask(image, 'red')

#	cv2.imshow('redMask', redMask)
#	cv2.waitKey(1)
#	return


	blueMask = mk.get_mask(image, 'blue')

	"""
	cv2.imshow('blueMask', blueMask)

	cv2.waitKey(1)
	return
	"""

	# Detecting red ball
	radiusRed, centreRed, showImg = mk.image_handler_function(image, True, redMask)

	# Detecting blue ball
	radiusBlue, centreBlue, showImg = mk.image_handler_function(image, True, blueMask)

	if centreRed != None:
		color = 'red'
		radius = radiusRed
		position = centreRed
		res = {'color': color, 'radius': radius, 'position': position}
		ball_list. append(res)
		global ball_radius
		ball_radius = radius
	if centreBlue != None:
		color = 'blue'
		radius = radiusBlue
		position = centreBlue
		res = {'color': color, 'radius': radius, 'position': position}
		ball_list. append(res)
		
	return ball_list

def processImages(im):
	
	# img = readImage( 'blue.jpg' )
	#leftImage = readImage( 'left.jpg' )
	#rightImage = readImage( 'right.jpg' )

	#print detectBall(im[0])
	#return

	global ball_radius
	ball_radius = 0
	res = getPositionWeights( im[0], im[1] ) #leftImage, rightImage )
	w = res["red"]
	
	return ( w, ball_radius )

	# img = readImage( 'Balls.jpg' )
	# img = readImage( 'no_ball.jpg' )
	# ball_list = detectBall( img )


	# print 'Algorithm result:'

	# if len(ball_list) == 0:
	# 	print("no balls")
	# else:
	# 	for res in ball_list:
	# 		print 'Color: ', res['color']
	# 		print 'Position: ', res['position']
	# 		print 'Radius: ', res['radius']
	# 
