#   Makeing the robot move foward and back
#   Dan Cohen
#   1/21/21
#
#
###############################

import rospy
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
import math
import time
#from std_srvs.srv import Empty


# Robot start at 0 for all values, POSE

x = 0
y = 0
yaw = 0

def poseCallback(pose_message):
    global x
    global y,yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

    #print( "pose callback funtion")
    #print(pose_message)


def move(speed,distace,is_foward):
    #declare a Twist msg to send vel commands
    # gather current location
    velocity_message = Twist()
    global x, y
    y0 = y
    x0 = x


    if(is_foward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    distance_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

    while True:
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()

        #rospy.Duration()


        distance_moved = distance_moved + abs(0.5 * math.sqrt(((x-x0)**2) + ((y-y0)**2)))
        #print("moved distace")
        print(distance_moved)
        if not (distance_moved<distace):
            rospy.loginfo("reached")
            break

        velocity_message.linear.x = 0
        velocity_publisher.publish(velocity_message)

if __name__ == '__main__':

    try:
        rospy.init_node('robot_cleaner',anonymous = True)
        print("started")
        #declare vel pub
        #velocity_message = Twist()
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        #velocity_message.linear.x = abs(1.0)
        #rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)


        position_topic = "/turtle/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)
        print("here")
        print("Lets move the robot")
        speed = input("what is your speed: ")
        distace = input("what is your distace: ")
        is_foward = input("Foward: True? ")
        move(speed,distace,is_foward)


    except rospy.ROSInterruptException:
        pass









