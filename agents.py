from game import *


class Agent:
    def __init__(self, index, isPacman=False):
        self.index = index
        self.isPacman = isPacman
        self.location = (-1, -1)
        self.prevLocation = (-1, -1)
        self.nextLocation = (-1, -1)

    def getMove(self, gameState):
        pass

    def getDirectionOfMove(self, location):
        if location[0] > self.location[0]:
            return Directions.EAST
        elif location[0] < self.location[0]:
            return Directions.WEST
        elif location[1] > self.location[1]:
            return Directions.NORTH
        elif location[1] < self.location[1]:
            return Directions.SOUTH
        else:
            return Directions.STOP

    def setLocation(self, location):
        self.prevLocation = self.location
        self.location = location


class PacmanAgent(Agent):
    def __init__(self, index):
        Agent.__init__(self, index, True)

    def getMove(self, gameState):
        pass


class GhostAgent(Agent):
    chaseMode = False

    def __init__(self, index):
        Agent.__init__(self, index)

    # returns the target node for the Ghost
    def getGoal(self, gameState):
        pass

    # returns the sequence of (row,column) grid locations on the path to the destination
    def getMove(self, gameState):
        destination = self.getGoal(gameState)
        startNode = gameState.gameBoard.getNode(self.location)

        # setup cost matrix
        assistantMatrix = {}
        for row in xrange(0, len(gameState.gameBoard.getBoardHeight())):
            for column in xrange(0, len(gameState.gameBoard.getBoardWidth())):
                assistantMatrix[(row, column)] = (10000, False, None)

        assistantMatrix[self.location] = 0, False, None
        assistantMatrix[(self.prevLocation)] = 10000, True, None

        openSet = []
        openSet.append(startNode)

        while openSet:
            current = openSet[0]
            for n in openSet:
                if assistantMatrix[n.location][0] + self.manhattan(n.location, destination.location) < \
                                assistantMatrix[current.location][0] + self.manhattan(current.location,
                                                                                      destination.location):
                    current = n

            if current.location == destination.location:
                return self.getNextMove(current.location, assistantMatrix)

            openSet.remove(current)
            assistantMatrix[current.location][1] = True

            for child in current.children:
                if not assistantMatrix[child.location][1]:
                    if child not in openSet:
                        openSet.append(child)

                    childCost = assistantMatrix[child.location][0]
                    currentCost = assistantMatrix[current.location][0]

                    if childCost > currentCost + 1:
                        assistantMatrix[child.location][0] = currentCost + 1
                        assistantMatrix[current.location][2] = current.location
        return []

    def manhattan(self, location1, location2):
        return abs(location1[0] - location2[0]) + abs(location1[1] - location2[1])

    def getNextMove(self, nodeLocation, assistantMatrix):
        prevLocation = None
        while assistantMatrix[nodeLocation][2]:
            prevLocation = nodeLocation
            nodeLocation = assistantMatrix[nodeLocation][2]
        return prevLocation


class AgentState:
    def __init__(self, location):
        self.location = location
        self.color = (255, 255, 255)
