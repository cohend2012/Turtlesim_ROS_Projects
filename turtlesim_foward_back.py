#   Makeing the robot move foward and back
#   Dan Cohen
#   1/21/21
#
#
###############################

#   Makeing the robot move foward and back
#   Dan Cohen
#   1/21/21
#
#
###############################


import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import time


PI = 3.1415926535897

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta,4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def angular_vel_rotate(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (goal_pose.theta - self.pose.theta)

    def getstarttheta(self):

        goal_pose = Pose()
        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        goal_pose.theta = self.steering_angle(goal_pose)
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        return goal_pose.theta, goal_pose.x, goal_pose.y

    def move2goal(self,move_x,move_y):
        """Moves the turtle to the goal."""
        move_x = move_x
        move_y = move_y
        #theta_goal_calc =
        goal_pose = Pose()
        if (move_x and move_y) == 0:
            goal_pose.x = goal_pose.x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        elif (move_x ==0 and move_y !=0):
            goal_pose.x = goal_pose.x
            goal_pose.y = move_y
            goal_pose.theta = self.steering_angle(goal_pose)

        elif(move_x !=0 and move_y ==0):
            goal_pose.x = move_x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        else:
            goal_pose.x = move_x
            goal_pose.y = move_y
            goal_pose.theta = goal_pose.theta


        #theta_goal_calc =
        # Get the input from the user.
        #goal_pose.x = float(input("Set your x goal: "))
        #goal_pose.y = float(input("Set your y goal: "))
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.01
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0 #self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("pose of the robot theta",self.pose.theta)
            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


        # If we press control + C, the node will stop.
       # rospy.spin()

    def turn(self,goal_pose):
        theta_input = goal_pose
        goal_pose = Pose() # rewrite it
        goal_pose.theta = theta_input
        #goal_pose.x = float(input("Set your x goal: "))
        #goal_pose.y = float(input("Set your y goal: "))
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))*2*PI/360
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #tolerance = float(input("Set your tolerance: "))
        tolerance = 0.0001
        #vel_msg = Twist()
        #while self.euclidean_distance(goal_pose) >= distance_tolerance:
        #angular_vel(self, goal_pose, constant=6)
        print(self.angular_vel_rotate(goal_pose))

        vel_msg = Twist()

        while abs(goal_pose.theta - self.pose.theta) >= tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel_rotate(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("pose of the robot theta", self.pose.theta)
            # Publish at the desired rate.
            self.rate.sleep()

            # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)




def poseCallback(pose_message):
    global x_robot
    global y_robot,yaw_robot
    x_robot = pose_message.x
    y_robot = pose_message.y
    yaw_robot = pose_message.theta



def rotate():
    global x_robot
    global y_robot, yaw_robot

    #y_robot_current = y_robot
    #x_robot_current = x_robot
    position_topic = "/turtle/pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
    #rate = rospy.Rate(10)
    Robot_pose=Pose()
    print(Robot_pose.theta)
    print(pose_subscriber,yaw_robot)

    #Starts a new node
    rospy.init_node('roate_robot', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()


    position_topic = "/turtle/pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
    #rate.sleep(2)
    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = input("Input your speed (degrees/sec):")
    angle = input("Type your distance (degrees):")
    clockwise = input("Clowkise?: ") #True or false

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):

        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    position_topic = "/turtle/pose"
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)



    print(pose_subscriber, yaw_robot)




def move():
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's move your robot")
    speed = input("Input your speed:")
    distance_x = input("Type your distance in the x direction:")
    distance_y = input("Type your distance in the y direction:")
    isForward = True

    # Checking if the movement is forward or backwards
    if (isForward):
        vel_msg.linear.x = abs(distance_x)
        vel_msg.linear.y = abs(distance_y)
    #else:
        #vel_msg.linear.x = -abs(speed)
    # Since we are moving just in x-axis or y-axis

    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    if(distance_x == 0):
        distance = distance_y
    else:
        distance = distance_x

    while True:

        # Setting the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0

        # Loop to move the turtle in an specified distance
        #current_distance = abs(0.5 * math.sqrt(((x - x0) ** 2) + ((y - y0) ** 2)))
        while (abs(current_distance - distance) >0.01):
            # Publish the velocity
            print(current_distance,distance)
            velocity_publisher.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = float(rospy.Time.now().to_sec())
            # Calculates distancePoseStamped

            current_distance = speed * (t1 - t0)


        # After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        # Force the robot to stop
        velocity_publisher.publish(vel_msg)
        break

if __name__ == '__main__':
    try:
        # Testing our function
        # Robot starts at
        x_robot = 5.544445
        y_robot = 5.544445
        yaw_robot = 0
        global a, b
        global x_starting,y_starting
        #a = input("How long is the short side of your path")
        #b = input("how long is the long side of the path")
        # x_starting = input("How long is the short side of your path")
        # y_starting = input("how long is the long side of the path")
        # User input the path specs
        x_starting =1.0
        y_starting = 1.0
        a = 5 # from left to right
        b = 0.5 # from boot to top
        #move()
        #rotate()
        x_count = 0
        y_count = 0
        rotate_count =0
        # Move the robot to the start of the path.
        x = TurtleBot()
        starting_theta,goal_x,goal_y = x.getstarttheta()
        print(starting_theta)
        print(goal_x,goal_y)
        #x.move2goal()
        x.turn(float(starting_theta))
        #move_x =
        #move_y =
        x.move2goal(goal_x, goal_y)
        x.turn(float(0.0))
        for turn in range(20):

            if (turn % 2) == 0 or turn == 0:
                if (x_count % 2 ==0 or x_count == 0):
                    goal_x = a + goal_x
                else:
                    goal_x = -a + goal_x

                print(goal_x,goal_y)
                #input_user = input()
                #goal_x = a + goal_x
                goal_y = goal_y
                x.move2goal(goal_x, goal_y)
                x_count = x_count + 1

            else:
                #print(goal_x,goal_y)

                goal_x = goal_x
                goal_y = b + goal_y
                #print(goal_x, goal_y)
                x.move2goal(goal_x, goal_y)
                y_count = y_count + 1
            print(goal_x, goal_y)

            print(turn)
            #testing_input = input()


            if (rotate_count ==0 or rotate_count ==2 ):

                x.turn(float(90.0 * 2 * PI / 360))

            elif(rotate_count ==1):

                x.turn(float(-180 * 2 * PI / 360))
            elif(rotate_count ==3):

                x.turn(float(-0 * 2 * PI / 360))

            rotate_count =rotate_count +1

            if (rotate_count >= 4):

                rotate_count = 0


            #if (turn % 2) == 0 or turn == 0:
               # x.turn(float(90.0*2*PI/360))

            #else:
                #x.turn(float(-180.0*2*PI/360))
            #rotate()

        starting_theta, goal_x, goal_y = x.getstarttheta()
        print(starting_theta)
        print(goal_x, goal_y)
        # x.move2goal()
        x.turn(float(starting_theta))

    except rospy.ROSInterruptException:
        pass














