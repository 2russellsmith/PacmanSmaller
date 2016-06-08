#!/usr/bin/python

import sys, rospy, math, cv2
from PyQt4 import QtGui, QtCore
from sphero_swarm_node.msg import SpheroTwist, SpheroColor
from multi_apriltags_tracker.msg import april_tag_pos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PacmanController import PacmanController

key_instruct_label = """
    Control Your Sphero!
    ---------------------------
    Moving around:
       u    i    o
       j    k    l
       m    ,    .
    """

BOARD_WIDTH = 19
BOARD_HEIGHT = 9

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600

CELL_WIDTH = CAMERA_WIDTH / BOARD_WIDTH
CELL_HEIGHT = CAMERA_HEIGHT / CAMERA_HEIGHT


def toDiscretized(location):
    x = int(location[0] / CELL_WIDTH)
    y = int(location[1] / CELL_HEIGHT)
    return x, y


def fromDiscretized(location):
    x = (location[0] + .5) * CELL_WIDTH
    y = (location[1] + .5) * CELL_HEIGHT
    return int(x), int(y)


class PacmanGui(QtGui.QWidget):
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480)

        self.stopFlag = True
        self.controller = None

        self.sphero_dict = rospy.get_param('/sphero_swarm/connected')

        #############
        # LISTENERS #
        #############

        # self.cmdVelPub is who we tell about to move sphero
        self.cmdVelPub = rospy.Publisher('cmd_vel', SpheroTwist, queue_size=1)

        # How to get camera feed, draw on it and publish it to a feed that the camera program can display
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/camera/image_raw", Image, self.cameraImageCallback, queue_size=1)
        self.publisher = rospy.Publisher("/output/image_raw", Image, queue_size=1)

        # who we tell if we want to update the color
        self.colorPub = rospy.Publisher('set_color', SpheroColor, queue_size=1)

        # aprtSub tells us when april tags are updated. When this happens the callback function is called.
        self.aprtSub = rospy.Subscriber('april_tag_pos', april_tag_pos, self.aprtCallback)

        ################
        # GUI ELEMENTS #
        ################

        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)

        self.controllerDropDown = QtGui.QComboBox()

        # Add your methods here
        self.controllerDropDown.addItem("Zeta Pacman")

        self.controllerStartBtn = QtGui.QPushButton("Start")
        self.controllerStartBtn.clicked.connect(self.start)

        self.controllerStopBtn = QtGui.QPushButton("Stop")
        self.controllerStopBtn.clicked.connect(self.stop)

        self.layout = QtGui.QVBoxLayout()
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.controllerDropDown)
        hlayout.addWidget(self.controllerStartBtn)
        hlayout.addWidget(self.controllerStopBtn)

        self.layout.addLayout(hlayout)
        self.setLayout(self.layout)
        self.setWindowTitle("AI Pacman")
        self.show()

        rospy.init_node('Pacman AI', anonymous=True)

    def start(self):
        currentMethod = self.aprilTagDropDown.currentText()

        if self.controller:
            self.stop()

        # Add an if statement for each item in the drop down to create your controller for that method
        if currentMethod == "Zeta Pacman":
            print "Zeta Pacman"
            self.controller = PacmanController()

        if self.controller:
            self.controller.startExecution()

    def stop(self):
        if self.controller:
            self.controller.stopExecution()

    def cameraImageCallback(self, ros_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")

            self.drawOverlay(cv_image)

            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def drawOverlay(self, image):

        # draw the grid
        controllerData = self.controller.getGUIData()

        # constants for variable drawing
        color = (0, 0, 255)
        thickness = 1
        lineType = 8
        shift = 0
        radius = 5
        for i in range(0, BOARD_HEIGHT):
            for j in range(0, BOARD_WIDTH):
                # draw boxes
                topLeftCorner = (CELL_WIDTH * i, CELL_HEIGHT * j)
                bottomRightCorner = (CELL_WIDTH * (i + 1), CELL_HEIGHT * (j + 1))
                cv2.rectangle(image, topLeftCorner, bottomRightCorner, color, thickness, lineType, shift)

                # draw pellets
                if (i, j) in controllerData['pellet_locations']:
                    cv2.circle(image, fromDiscretized((i, j)), radius, color, thickness, lineType, shift)
                    # http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # main body of algorithm should go here. MSG contains an id, x,y and orientation data members
    def aprtCallback(self, msg):
        print('april tag call back' + str(msg))

        # still loading
        if not self.stopFlag or self.controller is None:
            return

        # update the controller with the new tag locations
        discretizedLocations = [toDiscretized(x) for x in msg.pose]
        tagLocations = {x[0]: x[1] for x in list(zip(msg.ids, discretizedLocations))}
        self.controller.updateAgents(tagLocations)
        # for location, id in list(zip(msg.pose, msg.id)):
        #     tagLocations
        #
        # for location in self.location:
        #     self.location[location] = (-1, -1)
        #
        # # iterate through array of april-tag ids
        # for i in range(0, len(msg.id)):
        #     self.location[msg.id[i]] = (msg.pose[i].x, msg.pose[i].y)

        minX = -1
        minY = -1
        maxX = -1
        maxY = -1

        for spheroId in msg.id:
            agent = self.getAgent(spheroId)
            if agent is not None:
                agent.setLocation(self.toDiscretized(self.location[spheroId]))

                toHere = self.location[spheroId]
                if toHere[0] == -1:
                    continue
                nextIndex = self.order.index(spheroId) + 1
                if nextIndex >= len(self.order):
                    continue
                nextSphero = self.order[nextIndex]
                fromHere = self.location[nextSphero]
                if fromHere[0] == -1:
                    continue
                twist = self.getTwistFromDirection(agent.getMove(self.gameState))

                twist.name = self.numToSphero[nextSphero]
                self.cmdVelPub.publish(twist)  # how to tell sphero to move. all fields in twist must be explicitly set.
            else:
                location = self.location[spheroId]
                if minX == -1:
                    minX = location[0]
                    minY = location[1]
                    maxX = location[0]
                    maxY = location[1]
                if (location[0] < minX):
                    minX = location[0]
                if (location[1] < minY):
                    minY = location[1]
                if (location[0] > maxX):
                    maxX = location[0]
                if (location[1] < maxY):
                    maxY = location[1]

        # calculate the height and width of the board according to tag corner positions
        BOARD_WIDTH = maxX - minX
        BOARD_HEIGHT = maxY - minY
        # calculate the height and width of the boxes to be drawn
        BOX_WIDTH = BOARD_WIDTH / BOX_X_COUNT
        BOX_HEIGHT = BOARD_HEIGHT / BOX_Y_COUNT


    # TODO Can be used to manually drive pacman
    # These can be changed
    # http://doc.qt.io/qt-5/qt.html#Key-enum
    def keyPressEvent(self, e):
        twist = None
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80
            twist.linear.y = 80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80
            twist.linear.y = 80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80
            twist.linear.y = -80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = -80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80
            twist.linear.y = -80
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        if twist != None:
            self.cmdVelPub.publish(twist)


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    p = PacmanGui()
    p.show()
    sys.exit(app.exec_())
