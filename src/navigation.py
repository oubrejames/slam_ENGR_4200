#!/usr/bin/env python

# ROS general imports
import roslib
roslib.load_manifest('lab1_turtlebot')
import rospy


#ROS messages
#TODO import appropiate ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose


class Navigation(object):
    
    def __init__(self):
        '''
        constructor
        '''
        #Initialize ros node
        rospy.init_node('turtlebot_navigation')        
        
        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        
        
        #TODO: Define subscriber: topic name, message type, function callback
        self.sub = rospy.Subscriber('/odom',Odometry , self.callback)

        
        # TODO: Define pulisher: topic name, message type
        # Publish velocity to the bot       
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
        
        #TODO define the velocity message
        # could be turtle_actionlib/Velocity
        self.vmsg = Twist()     


    def dead_end(self):

        # Stop turtlebot

        # Rotate 180 degrees

        # Start moving again



    def No_New_Paths:

        # If at junction that has not been seen

        # If path on right, take it


        # If not, take path on left


        # If not, take path straight ahead



    def familiar_junction:

        # If junction has already been visited, turn back the way you came.

        self.dead_end()
        # If you are on a path that has been marked, take a new path if possible. 







