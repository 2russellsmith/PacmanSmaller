import sys, rospy, math, random
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info
from Lab1.tags import *
import json

from Lab2.RRTMaze import RRTMaze

class RRTKeyController:
    stop = True 
    tagList = []
    current_index = 0
    starting_location = None
    ending_location = None
    

    def __init__(self):
        self.stop = True
        self.wayPoint = None
        self.path = None
        self.reached_key = False

    def trackposCallback(self, msg):

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

                    if not self.reached_key:
                        self.colorPub.publish(ColorRGBA(255, 0, 0, 0))
                        self.reached_key = True
                        self.find_path(start=self.key, end=self.ending_location)

                    else:
                        self.stopExecution()          

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

    def find_path(self, start, end):

        maze = RRTMaze(polygons=self.poly_list, width=800, height=600, start=start, end=end)
        self.path = maze.generate_path()
        self.wayPoint = Attractive(center=self.path[0], t_id=0)
        self.current_index = 0


    def startExecution(self):
        rospy.wait_for_service("apriltags_info")

        info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
        self.resp = info_query()

        self.goal_tag_index = -1
        self.starting_location_index = -1
        self.key_index = -1
        self.key = None
        self.door_index = -1
        self.door = None
        self.poly_list = []

        for index, tag_id in enumerate(self.resp.ids):
            if tag_id == 0:
                self.goal_tag_index = index

            elif tag_id == 1:
                self.starting_location_index = index 

            elif tag_id == 2:
                self.key_index = index
                self.key = self.resp.polygons[index]

            elif tag_id == 20:
                self.door_index = index
                self.door = self.resp.polygons[index]

            else:
                self.poly_list.append(self.resp.polygons[index])


        key_tag = self.resp.polygons[self.key_index]
        key_x = sum([p.x for p in key_tag.points]) / len(key_tag.points)
        key_y = sum([p.y for p in key_tag.points]) / len(key_tag.points)

        ending_tag = self.resp.polygons[self.goal_tag_index]
        ending_x = sum([p.x for p in ending_tag.points]) / len(ending_tag.points)
        ending_y = sum([p.y for p in ending_tag.points]) / len(ending_tag.points)

        starting_tag = self.resp.polygons[self.starting_location_index]
        starting_x = sum([p.x for p in starting_tag.points]) / len(starting_tag.points)
        starting_y = sum([p.y for p in starting_tag.points]) / len(starting_tag.points)

        self.key = (key_x, key_y)
        self.ending_location = (ending_x, ending_y) 
        self.starting_location = (starting_x, starting_y)
        
        self.find_path(start=self.starting_location, end=self.key)
	self.stop = False

        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

        # Used to change the robot's color during runtime
        self.colorPub = rospy.Publisher('set_color', ColorRGBA, queue_size=1)

    def stopExecution(self):
        self.colorPub.publish(ColorRGBA(255, 255, 255, 0))
        self.stop = True
