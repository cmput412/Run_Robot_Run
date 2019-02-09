#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy, smach, smach_ros, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy, Image
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import Led, BumperEvent


class SleepState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Line','Done'])
        self.led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 1 )
        self.rate = rospy.Rate(10)  
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0                 # used to determine if the program should exit
        self.START = 0               # used to determine if the program should begin


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
            if self.START:
                return 'Line'
        return 'Done'


class LineFollow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Stop','Done'])
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',   
                        Image,self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.twist= Twist()
        self.rate = rospy.Rate(10)
        self.end = 0 
        self.stop = 0
        self.M = None
        self.RM = None
        self.image = None

    def execute(self, userdata):
        rospy.loginfo('Executing Line Follow state')
        self.stop = 0 
        self.twist = Twist()
        while not rospy.is_shutdown():
            if self.end:
                return 'Done'
            if self.stop:
                rospy.sleep(1)
                return 'Stop'
        return 'Done'

    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[1] == 1:
            self.end = 1

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([180,180,180])#[186,186,186])         # set upper and lower range for white mask
        upper_white = numpy.array([255,255,255])#[255,255,255])
        whitemask = cv2.inRange(self.image,lower_white,upper_white)

        lower_red = numpy.array([120,150,150])                          # set upper and lower range for red mask
        upper_red = numpy.array([180,255,255])
        redmask = cv2.inRange(hsv,lower_red,upper_red)

  
        h, w, d =self.image.shape
        search_top = 3*h/4
        search_bot = search_top + 20

        whitemask[0:search_top, 0:w] = 0                                # search for white color
        whitemask[search_bot:h, 0:w] = 0

        redmask[0:search_top, 0:w] = 0                                  # search for red color
        redmask[search_bot:h, 0:w] = 0

        self.M = cv2.moments(whitemask)
        self.RM = cv2.moments(redmask)

        if self.M['m00'] > 0:
            self.PID_Controller(w)

        if self.RM['m00'] > 0:
            self.stop = 1

        cv2.imshow("window", self.image)
        cv2.waitKey(3)

    def PID_Controller(self,w):
        prev_err = 0
        integral = 0
        dt = 1

        cx = int(self.M['m10']/self.M['m00'])
        cy = int(self.M['m01']/self.M['m00'])
        cv2.circle(self.image, (cx, cy), 20, (0,0,255),-1)
        err = cx - w/2
        Kp = .006
        Ki = 0
        Kd = .002
        integral = integral + err * dt
        derivative = (err-prev_err) / dt
        prev_err = err
        output = (err * Kp) + (integral * Ki) + (derivative * Kd)
        self.twist.linear.x = 0.3
        self.twist.angular.z =  -output
        self.cmd_vel_pub.publish(self.twist)




class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Line','Done'])
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0
        self.twist = Twist()

    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[1] == 1:
            self.end = 1

    def execute(self,userdata):
        rospy.loginfo('Executing Stop state')
        self.twist = Twist()
        while not rospy.is_shutdown():
            time = rospy.Time.now() + rospy.Duration(2)
            while rospy.Time.now() < time:
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
                if self.end:
                    return 'Done'
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)
            return 'Line'
        return 'Done'


class Turn90Clockwise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Line','Done'])
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0
        self.twist = Twist()
        self.speed = 90
        self.angle = 90
        self.angular_speed = self.speed*2*math.pi/360
        self.relative_angle = self.angle*3.3*math.pi/360


    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[1] == 1:
            self.end = 1

    def execute(self,userdata):
        rospy.loginfo('Executing Turn90 state')
        self.twist = Twist()
        while not rospy.is_shutdown():
            current_angle = 0
            self.twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.twist)
            t0 = rospy.Time.now().to_sec()
            while(current_angle < self.relative_angle):
                self.cmd_vel_pub.publish(self.twist)
                t1 = rospy.Time.now().to_sec()
                current_angle = self.angular_speed*(t1-t0)
                rospy.loginfo(current_angle)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            return 'Done'

            


        return 'Done'


class Turn90CounterClockwise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Line','Done'])
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0
        self.twist = Twist()
        self.speed = -90
        self.angle = 90
        self.angular_speed = self.speed*2*math.pi/360
        self.relative_angle = self.angle*3.3*math.pi/360


    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[1] == 1:
            self.end = 1

    def execute(self,userdata):
        rospy.loginfo('Executing Turn90 state')
        self.twist = Twist()
        while not rospy.is_shutdown():
            current_angle = 0
            self.twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.twist)
            t0 = rospy.Time.now().to_sec()
            while(current_angle < self.relative_angle):
                self.cmd_vel_pub.publish(self.twist)
                t1 = rospy.Time.now().to_sec()
                current_angle = abs(self.angular_speed)*(t1-t0)
                rospy.loginfo(current_angle)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            return 'Done'

            


        return 'Done'



class Turn180(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Line','Done'])
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                            Twist, queue_size=1)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0
        self.twist = Twist()
        self.speed = 90
        self.angle = 180
        self.angular_speed = self.speed*2*math.pi/360
        self.relative_angle = self.angle*3*math.pi/360


    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[1] == 1:
            self.end = 1

    def execute(self,userdata):
        rospy.loginfo('Executing Turn90 state')
        self.twist = Twist()
        while not rospy.is_shutdown():
            current_angle = 0
            self.twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.twist)
            t0 = rospy.Time.now().to_sec()
            while(current_angle < self.relative_angle):
                self.cmd_vel_pub.publish(self.twist)
                t1 = rospy.Time.now().to_sec()
                current_angle = self.angular_speed*(t1-t0)
                rospy.loginfo(current_angle)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            return 'Done'

            


        return 'Done'
    









def main():
    rospy.init_node('Comp2')
    rate = rospy.Rate(10)
    sm = smach.StateMachine(outcomes = ['DoneProgram'])
    sm.set_initial_state(['Turn180'])

    with sm:
        

        smach.StateMachine.add('SleepState', SleepState(),
                                        transitions = {'Line': 'LineFollow',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('LineFollow', LineFollow(),
                                        transitions = {'Stop': 'StopState',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('StopState', StopState(),
                                        transitions = {'Line': 'LineFollow',
                                                        'Done' : 'DoneProgram'})
        smach.StateMachine.add('Turn90Clockwise', Turn90Clockwise(),
                                        transitions = {'Line': 'LineFollow',
                                                        'Done' : 'DoneProgram'})
        smach.StateMachine.add('Turn90CounterClockwise', Turn90CounterClockwise(),
                                        transitions = {'Line': 'LineFollow',
                                                        'Done' : 'DoneProgram'})

        smach.StateMachine.add('Turn180', Turn180(),
                                        transitions = {'Line': 'LineFollow',
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
