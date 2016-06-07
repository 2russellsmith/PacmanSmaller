from agents import GhostAgent


class PinkGhost(GhostAgent):
    def __init__(self, index):
        GhostAgent.__init__(self, index)

    def getGoal(self, gameState):
        if GhostAgent.chaseMode:
            return gameState.gameBoard.getNode((0, 0))
        #Todo Set Goal to 3 spaces in front of where pac-man is facing
        return gameState.gameBoard.getNode(gameState.getPacman().location)

