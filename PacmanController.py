from game import GameState
from PinkGhost import PinkGhost
from RedGhost import RedGhost
from zetapacman import ZetaPacman


class PacmanController:
    PACMAN_ID = 0
    RED_GHOST_ID = 1
    PINK_GHOST_ID = 2

    def __init__(self):
        self.stop = True
        self.pacman = ZetaPacman(PacmanController.PACMAN_ID)
        self.redGhost = RedGhost(PacmanController.RED_GHOST_ID)
        self.pinkGhost = PinkGhost(PacmanController.PINK_GHOST_ID)
        self.agents = [self.pacman, self.redGhost, self.pinkGhost]
        self.gameState = GameState(self.agents)

    def startExecution(self):
        pass

    def stopExecution(self):
        pass

    def getGUIData(self):# todo this needs to be implemented with real values
        return {
            "score": 4,
            "pellet_locations": [(1,1), (5,5), (4,5)]
        }

    def updateAgents(self, tagLocations):

        pass