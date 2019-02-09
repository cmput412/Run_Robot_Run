#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, smach, smach_ros, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy, Image
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import Led, BumperEvent


class SleepState(smach.State):
    def __init__(self):
        self.START = 0
        smach.State.__init__(self, outcomes=['LineFollow','Done'])
        self.led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 1 )
        self.rate = rospy.Rate(10)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end


    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[0] == 1:
            self.START = 1
        if msg.buttons[1] == 1:
            self.end = 1

    def execute(self, userdata):
        rospy.loginfo('Executing sleep state')

        while not rospy.is_shutdown():
            if self.end:
                return 'Done'
            if self.START == 1:
                return 'LineFollow'
        return 'Done'


class LineFollow(smach.State):
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',   
                        Image,self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)

        self.twist= Twist()
        self.rate = rospy.Rate(10)
        self.time = rospy.Time.now()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([180,180,180])#[186,186,186])
        upper_white = numpy.array([255,255,255])#[255,255,255])
        whitemask = cv2.inRange(image,lower_white,upper_white)
        lower_red = numpy.array([120,150,150])
        upper_red = numpy.array([180,255,255])
        redmask = cv2.inRange(hsv,lower_red,upper_red)

  
        h, w, d =image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        whitemask[0:search_top, 0:w] = 0
        whitemask[search_bot:h, 0:w] = 0
        redmask[0:search_top, 0:w] = 0
        redmask[search_bot:h, 0:w] = 0
        M = cv2.moments(whitemask)
        RM = cv2.moments(redmask)
        prev_err = 0
        integral = 0
        dt = 1
        if rospy.Time.now() > self.time:
            self.twist.linear.x = 0.3
            self.cmd_vel_pub.publish(self.twist)

        if M['m00'] > 0 and (rospy.Time.now() > self.time):
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255),-1)
            err = cx - w/2
            Kp = .006
            \
            Ki = 0
            Kd = .002
            integral = integral + err * dt
            derivative = (err-prev_err) / dt
            prev_err = err
            output = (err * Kp) + (integral * Ki) + (derivative * Kd)
            self.twist.linear.x = 0.
            self.twist.angular.z =  -output#-float(err)/100
            self.cmd_vel_pub.publish(self.twist)

        elif RM['m00'] > 0 and (rospy.Time.now() > self.time):
            cx = int(RM['m10']/RM['m00'])
            cy = int(RM['m01']/RM['m00'])
            cv2.circle(image, (cx, cy), 20, (255,0,255),-1)
            self.twist = Twist()
            self.cmd_vel_pub.publish(self.twist)
            self.time = rospy.Time.now() + rospy.Duration(2)
        #cv2.resize("window", 500,500)
        cv2.imshow("window", image)
        cv2.waitKey(3)



class StopLine(smach.State):
    def __init__(self):
        pass

    









def main():
    rospy.init_node('Comp2')
    sm = smach.StateMachine(outcomes = ['DoneProgram'])
    sm.set_initial_state(['SleepState'])

    with sm:
        

        smach.StateMachine.add('SleepState', SleepState(),
                                        transitions = {'LineFollow': 'LineFollow',
                                                        'Done' : 'DoneProgram'})

         smach.StateMachine.add('LineFollow', LineFollow(),
                                        transitions = {'SleepState': 'SleepState',
                                                        'Done' : 'DoneProgram'})
 
 
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    

    sis.start()
    
    outcome = sm.execute() 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

'''def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print " parameter [%s] not defined, Defaulting to %.3f" % (name,default)
        return default'''



if __name__ == '__main__':
    main()
