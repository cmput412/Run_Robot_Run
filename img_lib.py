import rospy, cv2, cv_bridge
import argparse
import imutils

# code from https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
class ShapeDetector:
	def __init__(self):
		pass
 
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True) #compute perimeter
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        # if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"
 
		# if the shape has 4 vertices, it is either a square or
		# a rectangle
		elif len(approx) == 4:
			shape = "square"
 
		else:
			shape = "circle"
 
		# return the name of the shape
		return shape

def location1(img):
    """Counts number of red pillars and returns the number"""

    lower_red = numpy.array([120,150,150])                          # set upper and lower range for red mask
    upper_red = numpy.array([180,255,255])
    redmask = cv2.inRange(image,lower_red,upper_red)

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector()
    
    # Detect blobs.
    keypoints = detector.detect(im)

    return len(keypoints)

def location2(img):
    """Counts number of shapes and returns which shape is green"""

    #First, count the number of shapes
    lower_red = numpy.array([120,150,150])                          # set upper and lower range for red mask
    upper_red = numpy.array([180,255,255])
    redmask = cv2.inRange(image,lower_red,upper_red)

    # Set up the detector with default parameters.
    detector = cv2.SimpleBlobDetector()
    
    # Detect blobs.
    keypoints = detector.detect(im)
    num_red = len(keypoints)

    num_shape = num_red + 1

    #Now, determine which shape is green
    lower_green = numpy.array([15,120,90])                          # set upper and lower range for red mask
    upper_red = numpy.array([80,255,165])
    greenmask = cv2.inRange(image,lower_green,upper_green)

    gray = cv2.cvtColor(greenmask, cv2.COLOR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()

    # loop over the contours (in out case there should only be one)
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        shape = sd.detect(c)
    
    return num_shape, shape

def location3(img, shape):
    """Returns True if the current image displays the shape"""
    #First, count the number of shapes
    lower_red = numpy.array([120,150,150])                          # set upper and lower range for red mask
    upper_red = numpy.array([180,255,255])
    redmask = cv2.inRange(image,lower_red,upper_red)

    gray = cv2.cvtColor(redmask, cv2.COLOR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()

    # loop over the contours (in out case there should only be one)
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        detected_shape = sd.detect(c)

    if shape == detected_shape:
        return True
    
    else:
        return False