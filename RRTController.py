import sys, rospy, math, random
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info
from Lab1.tags import *
import json

from Lab2.RRTMaze import RRTMaze

# You implement this class
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

            #print "WAYPOINT: %s" % str(self.wayPoint.center)
            if self.wayPoint.goal(current):
                print "REACHED WAYPOINT"
                print "CURRENT LOCATION: %s" % str(current)
                self.current_index += 1

                if self.current_index < len(self.path):
                    self.wayPoint = Attractive(center=self.path[self.current_index], t_id=self.current_index)
                    print "PATH: %s" % str(self.path)
                    print "NEW WAY POINT: %s" % str(self.wayPoint.center)
                else:
                    print "REACHED GOAL"
                    self.stopExecution()          

            # Iterate over tagList and sum (X, Y) results of calcDelta on each
            delta_list = [tag.calcDelta(current) for tag in [self.wayPoint]]
            print delta_list
            final_delta = [sum(x) for x in zip(*delta_list)]

            #print "FINAL DELTA: %s" % final_delta

            # **Scaling Speed so it doesn't run off the other end of the tag**
            twist.linear.x = final_delta[0]
            twist.linear.y = final_delta[1]

            # Robot can't fly... yet.
            twist.linear.z = 0

            # Update twist values
            self.cmdVelPub.publish(twist)

    def find_path(self):

        polygons = self.resp.polygons
        del polygons[self.goal_tag_index]

        test = RRTMaze(polygons=polygons, width=800, height=600, start=self.starting_location, end=self.ending_location)

        self.path = test.generate_path()
        print "LENGTH OF PATH: %s" % len(self.path)
        print "".join([str(p) for p in self.path])

        self.wayPoint = Attractive(center=self.path[0], t_id=0)


    def startExecution(self):
        rospy.wait_for_service("apriltags_info")

        info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
        self.resp = info_query()

        self.goal_tag_index = -1
        self.starting_location_index = -1

        for index, tag_id in enumerate(self.resp.ids):
            if tag_id == 0:
                self.goal_tag_index = index

            if tag_id == 1:
                self.starting_location_index = index

            if self.goal_tag_index != -1 and self.starting_location_index != -1:
                break

        ending_tag = self.resp.polygons[self.goal_tag_index]
        ending_x = sum([p.x for p in ending_tag.points]) / len(ending_tag.points)
        ending_y = sum([p.y for p in ending_tag.points]) / len(ending_tag.points)

        starting_tag = self.resp.polygons[self.starting_location_index]
        starting_x = sum([p.x for p in starting_tag.points]) / len(starting_tag.points)
        starting_y = sum([p.y for p in starting_tag.points]) / len(starting_tag.points)

        self.ending_location = (ending_x, ending_y) 
        self.starting_location = (starting_x, starting_y)
        
        print str(self.starting_location)
        print str(self.ending_location)

        self.find_path()
	self.stop = False

        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        

    def stopExecution(self):
        self.stop = True
