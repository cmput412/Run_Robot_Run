#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, smach, smach_ros, time, math, img_lib
from sensor_msgs.msg import Image
from imutils import contours
from skimage import filters, morphology, measure
from kobuki_msgs.msg import Led
import imutils

numpy.set_printoptions(threshold=numpy.nan)


class CountourTest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done'])
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',   
                        Image,self.image_callback)
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 1 )
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size = 1 )
        self.first = 1
        self.count = 0
        self.grouping = 0
        self.i = 0
        self.avg = 0
        self.start = None
        self.end = None
        self.val = None
        

    def execute(self, userdata):
    	self.led1.publish(0)
        self.led2.publish(0)
    	self.start = rospy.Time.now()
        self.end = self.start + rospy.Duration(5)
        while not rospy.is_shutdown():
        	if rospy.Time.now() >= self.end:
    			self.val = self.avg
    			if self.val == 1:
    				rospy.loginfo('here1')
    				self.led1.publish(1)
    			elif self.val == 2:
    				rospy.loginfo('here2')
    				self.led1.publish(1)
    			else:
    				rospy.loginfo('here3')
    				self.led1.publish(1)
    				self.led2.publish(1)
    			rospy.loginfo(self.avg)
    			#return 'Done'
    		elif rospy.Time.now() < self.end:
    			rospy.loginfo('counting')



        return 'Done'

    def image_callback(self, msg):
    	
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #img_lib.location1(self.image)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    	#lower_red = numpy.array([120,30,100])#[100,0,0])                          # set upper and lower range for red mask
    	#upper_red = numpy.array([20,255,255])#[255,30,30])
    	#lower_red = numpy.array([20,0,0])#[100,0,0])                          # set upper and lower range for red mask
    	#upper_red = numpy.array([255,10,10])#[255,30,30])
    	#redmask = cv2.inRange(self.image,lower_red,upper_red)
    	#lower_red = numpy.array([100,100,100])
        #upper_red = numpy.array([255,255,255])
        redmask = self.threshold_hsv_360(30,80,20,255,255,120,hsv)
        #cv2.inRange(hsv,lower_red,upper_red)
    	#rospy.loginfo(redmask)
    	ret, thresh = cv2.threshold(redmask, 127, 255, 0)
    	im2, cnts, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    	cv2.drawContours(redmask, cnts, -1, (0,255,0), 3)
    	

    	#redmask2 = redmask + 1
    	#img = numpy.zeros((480,640))
    	#if self.first:
    	#	rospy.loginfo(redmask)
    	#for i in range(480):
    	img = measure.label(redmask, background=0)
    	#if self.first:
    	#	rospy.loginfo(2)
    	#	rospy.loginfo(img)
    	#	self.first = 0
    	img += 1
    	propsa = measure.regionprops(img.astype(int))
    	length = len(propsa)
    	self.grouping += length - 1
    	self.i += 1
    	self.avg = self.grouping/self.i
    	rospy.loginfo(self.avg)




    	#imgray = cv2.cvtColor(redmask, cv2.COLOR_BGR2GRAY)
    	#rospy.loginfo(imgray)
    	
    	#cnts = imutils.grab_contours(countours)

    	#rospy.loginfo(redmask[0])
    	#rospy.loginfo(redmask[0])
        #hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        cv2.imshow("window", redmask)
        cv2.waitKey(3)

    def threshold_hsv_360(self,s_min, v_min, h_max, s_max, v_max, h_min, hsv):
		lower_color_range_0 = numpy.array([0, s_min, v_min],dtype=float)
		upper_color_range_0 = numpy.array([h_max/2., s_max, v_max],dtype=float)
		lower_color_range_360 = numpy.array([h_min/2., s_min, v_min],dtype=float)
		upper_color_range_360 = numpy.array([360/2., s_max, v_max],dtype=float)
		mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
		mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
		mask = mask0 | mask360
		return mask



   

def main():
    rospy.init_node('Test')
    rate = rospy.Rate(10)
    sm = smach.StateMachine(outcomes = ['DoneProgram'])
    sm.set_initial_state(['CountourTest'])

    with sm:
        

        smach.StateMachine.add('CountourTest', CountourTest(),
                                        transitions = {'Done' : 'DoneProgram'})

 
 
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    

    sis.start()
    
    outcome = sm.execute() 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()




if __name__ == '__main__':
    main()