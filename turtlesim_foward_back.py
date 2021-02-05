

##################################################
# By: Dan Cohen
# Date: 1/21/2021
# ASU ID# 1204831845
# For class:SES 598 Assignment 1 Problem 2
# Prof. Dr. Das
# Program Name: turtlesim_foward_back.py
##################################################

# import the necessary libs
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt
import time


PI = 3.1415926535897  # Cost Pi

global hit_wall_flag  # bad idea global is bad in python but was used
global go_home        # do we want to go home
hit_wall_flag = False  # flag for hitting the wall
go_home = False        # flag for going home


#  creating a class
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
        self.pose = data  # input = pose data
        self.pose.x = round(self.pose.x, 4)  # pull off the x location
        self.pose.y = round(self.pose.y, 4)  # pull off the y location
        self.pose.theta = round(self.pose.theta,4) # get the theta abs



    def euclidean_distance(self, goal_pose):  # staright line diff
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5): # P controler
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

    def start_theta(self, x, y):
        goal_pose = Pose()  # set up the goal_pose
        goal_pose.x = x  # set the x pose
        goal_pose.y = y  # set the y pose
        goal_pose.theta = self.steering_angle(goal_pose)  # get the steering angle and then
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        return goal_pose

    def move2goal(self,move_x,move_y):
        """Moves the turtle to the goal."""
        move_x = move_x # not needed
        move_y = move_y
        #theta_goal_calc =
        goal_pose = Pose() # set up goal Pose as the pose set
        if (move_x and move_y) == 0:  # if you ask the robot to move the robot to 0,0
            goal_pose.x = goal_pose.x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        elif (move_x ==0 and move_y !=0):  # if x is 0 and y is not
            goal_pose.x = goal_pose.x
            goal_pose.y = move_y
            goal_pose.theta = self.steering_angle(goal_pose)

        elif(move_x !=0 and move_y ==0):  # if y is 0 but ther their is not
            goal_pose.x = move_x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        else:
            goal_pose.x = move_x  # otherwise use this way
            goal_pose.y = move_y
            goal_pose.theta = self.steering_angle(goal_pose)


        print("goal pose",goal_pose.x )


        #theta_goal_calc =
        # Get the input from the user.
        #goal_pose.x = float(input("Set your x goal: "))
        #goal_pose.y = float(input("Set your y goal: "))
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.1 # setting the distance tolerance
        vel_msg = Twist()
        global hit_wall_flag  # re-declaring the globals
        global go_home

        # setting them to false
        hit_wall_flag = False
        go_home = False
        # keep moving to the goal unless we have hit our tol or we hit the wall
        while self.euclidean_distance(goal_pose) >= distance_tolerance and hit_wall_flag == False :

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 1.0 #self.linear_vel(goal_pose) # robot moves at 1 m/s
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0 #self.angular_vel(goal_pose) # do not need to rotate to move

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("pose of the robot theta",self.pose.theta)
            print("pose gen msg ",self.pose)
            print("vel msg",vel_msg.linear.x)



            # if the robot hits the wall  and tell have not told it to go hom
            if((self.pose.x > 10.1 or self.pose.y > 10.1 or self.pose.x < 0.1 or self.pose.y < 0.1)  and go_home == False):
                print("wall")
                hit_wall_flag = True
                print("hit wall flag",hit_wall_flag)
                print("go home? True go home, false dont",go_home )
                go_home = True
                print("I am going home",go_home)






            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over or we end it due to a wall
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def move2home(self, move_x, move_y):

        #set up the goal_pose var then determine if you enterned a zero


        goal_pose = Pose()
        if (move_x and move_y) == 0:
            goal_pose.x = goal_pose.x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        elif (move_x == 0 and move_y != 0):
            goal_pose.x = goal_pose.x
            goal_pose.y = move_y
            goal_pose.theta = self.steering_angle(goal_pose)

        elif (move_x != 0 and move_y == 0):
            goal_pose.x = move_x
            goal_pose.y = goal_pose.y
            goal_pose.theta = goal_pose.theta

        else:
            goal_pose.x = move_x
            goal_pose.y = move_y
            goal_pose.theta = self.steering_angle(goal_pose)

        print("goal pose", goal_pose.x)

            # theta_goal_calc =
            # Get the input from the user.
            # goal_pose.x = float(input("Set your x goal: "))
            # goal_pose.y = float(input("Set your y goal: "))
            # goal_pose.theta = float(input("set your goal theta 0 is typ: "))
            # Please, insert a number slightly greater than 0 (e.g. 0.01).
            # distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.15 # dist tol
        vel_msg = Twist() # get the twist/ vel msg

        while self.euclidean_distance(goal_pose) >= distance_tolerance: # to send it home we just want to know if we
            # have it the goal tol

                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control

                # Linear velocity in the x-axis.
            vel_msg.linear.x = 1  # self.linear_vel(goal_pose) # cost vel now
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0  # self.angular_vel(goal_pose) # dont curve to the location

                # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("pose of the robot theta", self.pose.theta)
            print("pose gen msg ", self.pose)



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
    x_robot = pose_message.turtleBot
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
    vel_msg.linear.turtleBot=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.turtleBot = 0
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
        vel_msg.linear.turtleBot = abs(distance_x)
        vel_msg.linear.y = abs(distance_y)
    #else:
        #vel_msg.linear.x = -abs(speed)
    # Since we are moving just in x-axis or y-axis

    vel_msg.linear.z = 0
    vel_msg.angular.turtleBot = 0
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
        vel_msg.linear.turtleBot = 0
        vel_msg.linear.y = 0
        # Force the robot to stop
        velocity_publisher.publish(vel_msg)
        break

                              # Main Program
if __name__ == '__main__':
    try:
        # Testing our function
        # Robot starts at
        x_robot = 5.544445  # where the robot start, not needed
        y_robot = 5.544445
        yaw_robot = 0  #where the robot starts
        global a, b                          # making a few key parameters global. #bad idea in general for python
        global x_starting,y_starting
        #gather user input
        a = float(input("Set your a width "))
        b = float(input("Set your b length: "))

        #a = input("How long is the short side of your path")
        #b = input("how long is the long side of the path")
        # x_starting = input("How long is the short side of your path")
        # y_starting = input("how long is the long side of the path")
        # User input the path specs
        x_starting = 1.0
        y_starting = 1.0
        #a = 12 # from left to right
        #b = 0.5 # from boot to top
        #move()
        #rotate()
        # staring count
        x_count = 0
        y_count = 0
        rotate_count = 0
        # Move the robot to the start of the path.
        #calling Turtlebot class
        turtleBot = TurtleBot()
        #desiredPose = Pose()
        # set goal to start the robot to start
        goal_x = float(input("Set your x goal: "))
        goal_y = float(input("Set your y goal: "))
        # need to also take in a and b

        goal_pose = turtleBot.start_theta(goal_x, goal_y) # setting the goal pose
        print(goal_pose.theta,goal_x,goal_y)
        #x.move2goal()
        turtleBot.turn(float(goal_pose.theta))  # set the robot to turn to the right heading
        turtleBot.move2goal(goal_x, goal_y)  # set robot to move to the initial starting location
        turtleBot.turn(float(0.0))  # face the west wall

        turn_total = int(input("How many turns would you like to make? "))  # User needs to enter how many turns
        turn = 0

        mode_1 = True   # Start in mode one and not in mode 2
        mode_2 = False  # dont start in mode 2
        stop_program = False  # don't want to stop the program

        while(stop_program == False): # start and stop the program

            while(turn<=turn_total and mode_1 == True ): # could change that number

                if (turn % 2) == 0 or turn == 0: # if it is even or zero
                    if (x_count % 2 ==0 or x_count == 0): # is the count even
                        goal_x = a + goal_x # add or sub a
                    else:
                        goal_x = -a + goal_x

                    print(goal_x,goal_y)
                    #input_user = input()
                    #goal_x = a + goal_x
                    goal_y = goal_y
                    turtleBot.move2goal(goal_x, goal_y)  # send the robot to the location
                    x_count = x_count + 1  # add to the count

                else:
                    #print(goal_x,goal_y)

                    goal_x = goal_x
                    goal_y = b + goal_y
                    #print(goal_x, goal_y)
                    turtleBot.move2goal(goal_x, goal_y)
                    y_count = y_count + 1
                print(goal_x, goal_y)

                if (hit_wall_flag == True):
                   # flag_input = input()

                    #print("You hit a wall, in the loop we should go home)
                    go_home = True
                    print(go_home)
                    goal_x = float(input("Set your x goal: "))
                    goal_y = float(input("Set your y goal: "))
                    # get goal pose
                    goal_pose = turtleBot.start_theta(goal_x, goal_y)
                    print(goal_pose.theta, goal_x, goal_y)

                    turtleBot.turn(float(goal_pose.theta))

                    turtleBot.move2home(goal_x, goal_y)

                    rotate_count = 0
                    turn = 100
                    x_count = 0
                    y_count = 0

                    mode_1 = False
                    mode_2 = True
                    print("mode_1",mode_1)

                    print(turn)
                    break





                print(turn)
                #testing_input = input()


                if (rotate_count ==0 or rotate_count ==2 ):

                    turtleBot.turn(float(90.0 * 2 * PI / 360)) # north or up

                elif(rotate_count ==1):

                    turtleBot.turn(float(-180 * 2 * PI / 360)) # west or left
                elif(rotate_count ==3):

                    turtleBot.turn(float(-0 * 2 * PI / 360)) # East or right

                rotate_count =rotate_count +1

                if (rotate_count >= 4):

                    rotate_count = 0
                print(goal_x)

                turn = turn + 1
                mode_2 = True

            while(mode_2 == True):
                print("You hit a wall, out of loop or you have finished your pattern")

                goal_x = float(input("Set your x goal: "))
                goal_y = float(input("Set your y goal: "))

                goal_pose = turtleBot.start_theta(goal_x, goal_y)
                print(goal_pose.theta, goal_x, goal_y)
                # x.move2goal()
                turtleBot.turn(float(goal_pose.theta))
                print("We are going thome and should go to location x, y", goal_x , goal_y)
                turtleBot.move2home(goal_x, goal_y)
                turtleBot.turn(float(0.0))
                #turn_total = 20
                turn_total = int(input("How many turns would you like to make? "))  # User needs to enter how many turns
                rotate_count = 0
                turn = 0
                x_count = 0
                y_count = 0
                print("We need to edit the length and the width of the pattern")
                print("a = ",a)
                print("b = ",b)
                print("We recommend a home position of 1,1 and a < 10- home_x and b <10 - home_y (0.5) is good ")
                a = float(input("What value of a would you like now? "))
                b = float(input("What value of b would you like now? "))
                mode_1 = True
                mode_2 = False
                print("mode_1 : ",mode_1)



    except rospy.ROSInterruptException:
        pass














