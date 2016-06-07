import sys, rospy, math, random
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info
from Lab1.tags import *
import json

from Lab2.RRTMaze import RRTMaze

class RRTController:
    stop = True 
    tagList = []
    current_index = 0
    starting_location = None
    ending_location = None
    

    def __init__(self):
        self.stop = True
        self.wayPoint = None
        self.path = None

    # This function is continuously called
    def trackposCallback(self, msg):

        # Pass in robot's current location to calcDelta
        current = (msg.x, msg.y)

        if not self.stop:
            twist = Twist()

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            if self.wayPoint.goal(current):
                self.current_index += 1

                if self.current_index < len(self.path):
                    self.wayPoint = Attractive(center=self.path[self.current_index], t_id=self.current_index)
                else:
                    self.stopExecution()          

            # Iterate over tagList and sum (X, Y) results of calcDelta on each
            final_delta = self.wayPoint.calcDelta(current)

            max_velocity = max(final_delta[0], final_delta[1])
            scale_factor = 1

            # **Scaling Speed so it doesn't run off the other end of the tag**
            if max_velocity > 20:
                scale_factor = 20 / max_velocity
            
            twist.linear.x = final_delta[0] * scale_factor
            twist.linear.y = final_delta[1] * scale_factor
            

            # Robot can't fly... yet.
            twist.linear.z = 0

            # Update twist values
            self.cmdVelPub.publish(twist)

    def find_path(self):

        maze = RRTMaze(polygons=self.poly_list, width=800, height=600, start=self.starting_location, end=self.ending_location)
        self.path = maze.generate_path()
        self.wayPoint = Attractive(center=self.path[0], t_id=0)


    def startExecution(self):
        rospy.wait_for_service("apriltags_info")

        info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
        self.resp = info_query()

        self.goal_tag_index = -1
        self.starting_location_index = -1
        self.poly_list = []

        for index, tag_id in enumerate(self.resp.ids):
            if tag_id == 0:
                self.goal_tag_index = index

            elif tag_id == 1:
                self.starting_location_index = index 

            else:
                self.poly_list.append(self.resp.polygons[index])

        ending_tag = self.resp.polygons[self.goal_tag_index]
        ending_x = sum([p.x for p in ending_tag.points]) / len(ending_tag.points)
        ending_y = sum([p.y for p in ending_tag.points]) / len(ending_tag.points)

        starting_tag = self.resp.polygons[self.starting_location_index]
        starting_x = sum([p.x for p in starting_tag.points]) / len(starting_tag.points)
        starting_y = sum([p.y for p in starting_tag.points]) / len(starting_tag.points)

        self.ending_location = (ending_x, ending_y) 
        self.starting_location = (starting_x, starting_y)

        self.find_path()

        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        self.stop = False
        

    def stopExecution(self):
        self.stop = True
