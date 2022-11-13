import util 
from util import Belief, pdf, xToCol, yToRow 
from engine.const import Const
import random
import math
# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb() 
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #   
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    #   
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!
        
    ###################################################################################

    def printgrid(self, positions):
        l = [[0 for i in range(self.getBelief().getNumCols())] for j in range(self.getBelief().getNumRows())]
        for position in positions:
            l[position[0]][position[1]] += 1
        for row in l:
            for num in row:
                print(num, end = ",")
            print()
        print("Done")
    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        
        belief = self.getBelief()
        if isParked:
            return
        total_particles = 1000
        positions = [(0, 0) for i in range(total_particles)]
        # positions stores position of ith particle (x : row, y : column)
        current_particle = 0
        tupleWiseProbs = {}
        for quadruple in self.transProb:
            if quadruple[0] not in tupleWiseProbs:
                tupleWiseProbs[quadruple[0]] = {}
            tupleWiseProbs[quadruple[0]][quadruple[1]] = self.transProb[quadruple]
        for i in range(belief.getNumRows()):
            for j in range(belief.getNumCols()):
                if current_particle == total_particles:
                    break
                value = int(belief.getProb(i, j)*total_particles)
                for _ in range(value):
                    positions[current_particle] = (i, j)
                    current_particle += 1
                    if current_particle == total_particles:
                        break
        for i in range(current_particle, total_particles):
            positions[i] = (random.randint(0, self.belief.getNumRows()-1), random.randint(0, self.belief.getNumCols()-1))
        self.printgrid(positions)
        for i in range(total_particles):
            if positions[i] not in tupleWiseProbs:
                continue
            val = random.uniform(0, 1)
            for position in tupleWiseProbs[positions[i]]:
                if val < tupleWiseProbs[positions[i]][position]:
                    positions[i] = position
                    break
                else:
                    val -= tupleWiseProbs[positions[i]][position]
        self.printgrid(positions)
        positional_sum = {}
        for i in range(total_particles):
            # print(pdf(math.sqrt((yToRow(posY)-positions[i][0])**2 + (xToCol(posX)-positions[i][1])**2), Const.SONAR_STD, observedDist))
            if positions[i] in positional_sum:
                positional_sum[positions[i]] += pdf(math.sqrt((yToRow(posY)-positions[i][0])**2 + (xToCol(posX)-positions[i][1])**2), 5*Const.SONAR_STD, observedDist)
            else :
                positional_sum[positions[i]] = pdf(math.sqrt((yToRow(posY)-positions[i][0])**2 + (xToCol(posX)-positions[i][1])**2), 5*Const.SONAR_STD, observedDist)

        for i in range(belief.getNumRows()):
            for j in range(belief.getNumCols()):
                if (i, j) in positional_sum:
                    self.belief.setProb(i, j, positional_sum[(i, j)])
                else :
                    self.belief.setProb(i, j, 0)
        self.belief.normalize()
        print(self.belief.grid)
        return

    def getBelief(self) -> Belief:
        return self.belief