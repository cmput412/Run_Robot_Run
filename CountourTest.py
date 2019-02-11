#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, smach, smach_ros, time, math, img_lib
from sensor_msgs.msg import Image
import imutils


class CountourTest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done'])
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',   
                        Image,self.image_callback)
        self.first = 1
        self.count = 0

    def execute(self, userdata):
        while not rospy.is_shutdown():
            pass
        return 'Done'

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #img_lib.location1(self.image)


    	lower_red = numpy.array([20,0,0])#[100,0,0])                          # set upper and lower range for red mask
    	upper_red = numpy.array([300,255,255])#[255,30,30])
    	redmask = cv2.inRange(self.image,lower_red,upper_red)


    	#imgray = cv2.cvtColor(redmask, cv2.COLOR_BGR2GRAY)
    	#rospy.loginfo(imgray)
    	ret, thresh = cv2.threshold(redmask, 127, 255, 0)
    	im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    	cv2.drawContours(redmask, contours, -1, (0,255,0), 3)
    	#cnts = imutils.grab_contours(countours)

    	if len(contours) < 60:
    		self.count = 1
    	elif len(contours) < 120:
    		self.count = 2
    	else:
    		self.count = 3
    	#cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x))
    	#rospy.loginfo(redmask[0])
    	#rospy.loginfo(redmask[0])
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        cv2.imshow("window", redmask)
        cv2.waitKey(3)



   

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