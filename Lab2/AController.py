import sys, rospy, math, random
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info

from Lab1.tags import *

# You implement this class
class AController:
    stop = True # This is set to true when the stop button is pressed
    tagList = []

    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)

    # This function is continuously called
    def trackposCallback(self, msg):

        if not self.stop:
            twist = Twist()

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            
            # Pass in robot's current location to calcDelta
            current = (msg.x, msg.y)
            for tag in self.tagList:
                if tag.goal(current):
                    print "Step complete" 
                    if self.setNextAttractive():
                        self.stop = False
                    else:
                        self.stop = True
                    
  
            # Iterate over tagList and sum (X, Y) results of calcDelta on each
            delta_list = [tag.calcDelta(current) for tag in self.tagList]
            final_delta = [sum(x) for x in zip(*delta_list)]
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

    def expand_polygon(self, polygon):
        scale_factor = 1.5
        average_x = sum([item.x for item in polygon.points]) / len(polygon.points)
        average_y = sum([item.y for item in polygon.points]) / len(polygon.points)

        for point in polygon.points:
            point.x -= average_x
            point.y -= average_y
            point.x *= scale_factor
            point.y *= scale_factor
            point.x += average_x
            point.y += average_y

    def startExecution(self):
        
        # Reset tag list so that moved tags don't keep getting appended to the list
        self.scale = 20
        xSize = 800/self.scale
        ySize = 600/self.scale
        self.maze = []
        for i in range(xSize):
            row = []
            for j in range(ySize):
                row.append(Node())
            self.maze.append(row)
        #self.maze = [[self.Node() for i in range(ySize)] for j in range(xSize)]
        self.tagList = []
        self.highestID = 0
        self.currentID = 0
        rospy.wait_for_service("apriltags_info")

        info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
        resp = info_query()

        numberOfTags = len(resp.polygons)

        # Iterate over detected AprilTags
        for i in range(numberOfTags):
            # A polygon (access points using poly.points)
            poly = resp.polygons[i]
            self.expand_polygon(poly)
            # The polygon's id (just an integer, 0 is goal (attractive field), all else is bad (repulsive field))
            t_id = resp.ids[i]

            if t_id == 1: 
                 self.goalX = int(sum(p.x for p in poly.points)/4/self.scale)
                 self.goalY = int(sum(p.y for p in poly.points)/4/self.scale)
                 self.maze[self.goalX][self.goalY].goal = True
            elif t_id == 0:
                 self.startX = int(sum(p.x for p in poly.points)/4/self.scale)
                 self.startY = int(sum(p.y for p in poly.points)/4/self.scale)
            else:
                for x in range(int(min(p.x for p in poly.points)/self.scale), int(max(p.x for p in poly.points)/self.scale)):
                    for y in range(int(min(p.y for p in poly.points)/self.scale), int(max(p.y for p in poly.points)/self.scale)):
                        self.maze[x][y].wall = True
        print "Building Maze...."
        print xSize
        print ySize
        for x in range(xSize):
            for y in range(ySize):
                if self.maze[x][y].wall == False:
                    self.maze[x][y].x = x
                    self.maze[x][y].y = y
                    self.maze[x][y].heuristic = self.manhattan(self.maze[x][y])
                    if x != 0 and self.maze[x-1][y].wall == False:
                        self.maze[x][y].children.append(self.maze[x-1][y])
                    if x != xSize-1 and self.maze[x+1][y].wall == False:
                        self.maze[x][y].children.append(self.maze[x+1][y])
                    if y != 0 and self.maze[x][y-1].wall == False:
                        self.maze[x][y].children.append(self.maze[x][y-1])
                    if y != ySize-1 and self.maze[x][y+1].wall == False:
                        self.maze[x][y].children.append(self.maze[x][y+1])
        print "Maze built!!!"
        self.path = self.getPath()
        print "Printing Path...."
        self.print_path()
        print "Done Finding Path!!!!!!"
        self.tagList.append(Attractive(self.path[0][0], 0))
        self.stop = False

    def getPath(self):
        node = self.maze[self.startX][self.startY]
        openSet = []
        node.currentCost = 0
        openSet.append(node)
        while openSet:
            current = openSet[0]
            for n in openSet:
                if n.currentCost + n.heuristic < current.currentCost + current.heuristic:
                    current = n 
            if current.goal == True:  # TODO
                return self.constructPath(current) 
            openSet.remove(current)
            current.visited = True
            for child in current.children:
                if child.visited == False:
                    if child not in openSet:
                        openSet.append(child)

                    if child.currentCost > current.currentCost + 1:
                        child.currentCost = current.currentCost + 1 
                        child.parent = current
        return []
                    
    def constructPath(self, node):
        path = []
        result = [(node.x*self.scale, node.y*self.scale)]
        if hasattr(node, 'parent'):
            path = self.constructPath(node.parent)
            path.append(result)
        return path

    def manhattan(self, node):
        return abs(node.x - self.goalX) + abs(node.y - self.goalY)

    def stopExecution(self):
        self.stop = True

    def setNextAttractive(self):
        if self.path:
            self.tagList[0] = Attractive(self.path[0][0], 0)
            self.tagList[0].radius = self.scale
            self.path.pop(0)
            return True
        else:
            return False

    def print_maze(self):
        for row in self.maze:
            for e in row:
                print e,
            print

    def print_path(self):
        for e in self.path:
            print e



class Node:
    def __init__(self):
        self.children = []
        self.wall = False
        self.goal = False
        self.visited = False
        self.currentCost = 100000


