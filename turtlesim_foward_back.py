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

global hit_wall_flag
global go_home
hit_wall_flag = False
go_home = False

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
            goal_pose.theta = self.steering_angle(goal_pose)


        print("goal pose",goal_pose.x )


        #theta_goal_calc =
        # Get the input from the user.
        #goal_pose.x = float(input("Set your x goal: "))
        #goal_pose.y = float(input("Set your y goal: "))
        #goal_pose.theta = float(input("set your goal theta 0 is typ: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(input("Set your tolerance: "))
        distance_tolerance = 0.1
        vel_msg = Twist()
        global hit_wall_flag
        global go_home

        hit_wall_flag = False
        go_home = False

        while self.euclidean_distance(goal_pose) >= distance_tolerance and hit_wall_flag == False :

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0.7 #self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0 #self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            print("pose of the robot theta",self.pose.theta)
            print("pose gen msg ",self.pose)

            if((self.pose.x > 10.1 or self.pose.y > 10.1) and go_home == False):
                print("wall")
                hit_wall_flag = True
                print("hit wall flag",hit_wall_flag)
                print("go home? True go home, false dont",go_home )
                go_home = True
                print("I am going home",go_home)





            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def move2home(self, move_x, move_y):



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
        distance_tolerance = 0.1
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control

                # Linear velocity in the x-axis.
            vel_msg.linear.x = 0.7  # self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0  # self.angular_vel(goal_pose)

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
        a = 12 # from left to right
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
        turn_total = 20
        turn = 0

        mode_1 = True
        mode_2 = False
        stop_program = False

        while(stop_program == False):

            while(turn<=20 and mode_1 == True ):

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

                if (hit_wall_flag == True):
                   # flag_input = input()

                    #print("You hit a wall, in the loop we should go home)
                    go_home = True
                    print(go_home)
                    starting_theta, goal_x, goal_y = x.getstarttheta()
                    print(starting_theta)
                    print(goal_x, goal_y)
                    # x.move2goal()
                    x.turn(float(starting_theta))
                    x.move2goal(goal_x, goal_y)
                    #x.turn(float(0.0))
                    turn_total = 20
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

                    x.turn(float(90.0 * 2 * PI / 360)) # north or up

                elif(rotate_count ==1):

                    x.turn(float(-180 * 2 * PI / 360)) # west or left
                elif(rotate_count ==3):

                    x.turn(float(-0 * 2 * PI / 360)) # East or right

                rotate_count =rotate_count +1

                if (rotate_count >= 4):

                    rotate_count = 0
                print(goal_x)

                turn = turn + 1
                mode_2 = True

            while(mode_2 == True):
                print("You hit a wall, out of loop or you have fnished your pattern")
                starting_theta, goal_x, goal_y = x.getstarttheta()
                print(starting_theta)
                print(goal_x, goal_y)
                # x.move2goal()
                x.turn(float(starting_theta))
                # move_x =
                # move_y =
                print("We are going thome and should go to location x, y", goal_x , goal_y)
                x.move2home(goal_x, goal_y)
                x.turn(float(0.0))
                turn_total = 20
                rotate_count = 0
                turn = 0
                x_count = 0
                y_count = 0
                print("We need to edit the length and the width of the pattern")
                print("a = ",a)
                print("b = ",b)
                print("We recomend a home postion of 1,1 and a < 10- home_x and b <10 - home_y (0.5) is good ")
                a = float(input("What value of a would you like now? "))
                b = float(input("What value of b would you like now? "))
                mode_1 = True
                mode_2 = False
                print("mode_1 : ",mode_1)




            #input_sense = input()
            #if (goal_x or goal_y) > 10.1 or (goal_x or goal_y) < 0.01:
                # need to fix this
                #print("you have hit the wall, that is not good, we need to look at our options")
                #starting_theta, goal_x, goal_y = x.getstarttheta()
                #print(starting_theta)
                #print(goal_x, goal_y)
                # x.move2goal()
                #x.turn(float(starting_theta))
                # move_x =
                # move_y =
                #x.move2goal(goal_x, goal_y)

            #if (turn % 2) == 0 or turn == 0:
               # x.turn(float(90.0*2*PI/360))

            #else:
                #x.turn(float(-180.0*2*PI/360))
            #rotate()

        #starting_theta, goal_x, goal_y = x.getstarttheta()
        #print(starting_theta)
        #print(goal_x, goal_y)
        # x.move2goal()
        #x.turn(float(starting_theta))

    except rospy.ROSInterruptException:
        pass














