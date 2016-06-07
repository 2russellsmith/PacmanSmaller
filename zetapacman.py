from agents import PacmanAgent
from PinkGhost import PinkGhost
from game import GameState


class RewardsMap(object):
    def __init__(self, gameState=None):
        if gameState is not None:
            self.rewardsMap = self.buildMap(gameState)
        else:
            self.rewardsMap = []

    def __getitem__(self, item):
        return self.rewardsMap[item]

    def __str__(self):
        final = ""
        for row in range(len(self.rewardsMap)):
            r = " ".join([str(int(x)).center(5) for x in self.rewardsMap[row]])
            final += r
            final += "\n"

        return final

    def getValueAt(self, row, col):
        return self.rewardsMap[row][col]

    def _clone(self):
        newRewardsMap = RewardsMap()

        rewardsMap = []
        for x in range(len(self.rewardsMap)):
            row = []
            for y in range(len(self.rewardsMap[x])):
                row.append(self.rewardsMap[x][y])
            rewardsMap.append(row)

        newRewardsMap.rewardsMap = rewardsMap

        return newRewardsMap

    def buildMap(self, gameState):
        pass

    # this will not validate input reward map size
    def __add__(self, rewardsMap2):
        newRewardsMap = rewardsMap2._clone()

        for x in range(len(rewardsMap2.rewardsMap)):
            for y in range(len(rewardsMap2.rewardsMap[x])):
                newRewardsMap.rewardsMap[x][y] += self.rewardsMap[x][y]

        return newRewardsMap


class PelletMap(RewardsMap):
    pelletReward = 100

    def __init__(self, gameState):
        RewardsMap.__init__(self, gameState)

    def buildMap(self, gameState):
        rewardsMap = []

        for rowIndex in range(len(gameState.gameBoard._board)):
            row = []
            for columnIndex in range(len(gameState.gameBoard._board[rowIndex])):
                if gameState.gameBoard.hasFood(rowIndex, columnIndex):
                    row.append(PelletMap.pelletReward)
                else:
                    row.append(0)
            rewardsMap.append(row)

        return rewardsMap


class GhostMap(RewardsMap):
    ghostPenalty = -1000
    totalIterations = 5

    def __init__(self, gameState, ghost):
        self.ghost = ghost
        RewardsMap.__init__(self, gameState)

    def recBuild(self, heatMap, gameBoard, previousLocation, currentLocation, encounteredBranches=1, remainingIterations=totalIterations):
        if remainingIterations == 0:
            return heatMap

        row, col = currentLocation
        availableMoves = gameBoard.getNeighborCoordinates(row=row, col=col)

        if previousLocation in availableMoves:
            availableMoves.remove(previousLocation)

        distanceTraveled = GhostMap.totalIterations - remainingIterations
        heatMap[row][col] = GhostMap.ghostPenalty / (encounteredBranches * distanceTraveled + 1)

        childEncounteredBranches = encounteredBranches + len(availableMoves) - 1

        for move in availableMoves:
            heatMap = self.recBuild(heatMap=heatMap,
                                    gameBoard=gameBoard,
                                    previousLocation=currentLocation,
                                    currentLocation=move,
                                    encounteredBranches=childEncounteredBranches,
                                    remainingIterations=remainingIterations-1)

        return heatMap

    def buildMap(self, gameState):
        # build basic map
        rewardsMap = []
        for rowIndex in range(len(gameState.gameBoard._board)):
            rewardsMap.append([0] * len(gameState.gameBoard._board[rowIndex]))

        return self.recBuild(rewardsMap, gameState.gameBoard, self.ghost.prevLocation, self.ghost.location)


class GhostPitMap(RewardsMap):
    ghostPitPenalty = -100000000

    def __init__(self, gameState):
        # TODO Could parse map file and use locations with empty spaces
        self.pitLocations = [(3, 8), (3, 9), (3, 10), (4, 7), (4, 8), (4, 9), (4, 10), (4, 11)]
        RewardsMap.__init__(self, gameState)

    def buildMap(self, gameState):
        rewardsMap = []
        for rowIndex in range(len(gameState.gameBoard._board)):
            rewardsMap.append([0] * len(gameState.gameBoard._board[rowIndex]))

        for row, col in self.pitLocations:
            rewardsMap[row][col] = GhostPitMap.ghostPitPenalty

        return rewardsMap


class ZetaPacman(PacmanAgent):
    def __init__(self, index=0):
        PacmanAgent.__init__(self, index=index)

    def getMove(self, gameState):
        finalMap = PelletMap(gameState)

        for ghost in gameState.getGhosts():
            g = GhostMap(gameState, ghost)
            finalMap += g

        finalMap += GhostPitMap(gameState)
        row, col = self.location
        availableMoves = gameState.gameBoard.getNeighborCoordinates(row, col)
        return max(availableMoves, key=lambda x: finalMap[x[0]][x[1]])

if __name__ == '__main__':

    ghost1 = PinkGhost(index=1)
    ghost1.prevLocation = (6, 7)
    ghost1.location = (6, 8)

    ghost2 = PinkGhost(index=1)
    ghost2.prevLocation = (6, 2)
    ghost2.location = (6, 3)

    z = ZetaPacman()
    z.location = (6, 6)
    g = GameState([ghost1, ghost2, z])

    print(z.getMove(g))
