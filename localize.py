#!/usr/bin/env python3
#
#   hw5_localize.py
#
#   Homework 5 code framework to localize a robot in a grid...
#
#   Places to edit are marked as FIXME.
#
import numpy as np
from copy import deepcopy

from visualization import Visualization, Robot

from mazelib import Maze
from mazelib.generate.Prims import Prims

#
#  Define the Walls

Maze.set_seed(42)
size = 20

m = Maze()
m.generator = Prims(size // 2, size // 2)
m.generate()

walls = m.grid
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)

print(walls)

# generates start and end, if want them on outer wall its true
m.generate_entrances(True, True)
start = m.start
goal = m.end

# Make sure the start and end not on outer wall
if start[0] == 0:
    start = (start[0] + 1, start[1])
elif start[0] == rows - 1:
    start = (start[0] - 1, start[1])

if start[1] == 0:
    start = (start[0], start[1] + 1)
elif start[1] == cols - 1:
    start = (start[0], start[1] - 1)

#
#  Prediction
#
#    bel         Grid of probabilities (current belief)
#    drow, dcol  Delta in row/col
#    probCmd     Modeled probability of command executing
#    prd         Grid of probabilities (prediction)
#
def computePrediction(bel, drow, dcol, probCmd = 1):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols))

    # Determine the new probablities (remember to consider walls).
    for r in range(1, rows-1):
        for c in range(1, cols-1):
            new_r = r + drow
            new_c = c + dcol
            
            # If not colliding with a wall
            if walls[new_r, new_c] == 0:
                prd[new_r, new_c] += probCmd * bel[r, c]
                
                # Account for probability of command not executing
                prd[r, c] += (1 - probCmd) * bel[r, c]
            else:
                prd[r, c] += probCmd * bel[r, c]
    
    # Return the prediction grid
    return prd


#
#  Measurement Update (Correction)
#
#    prior       Grid of prior probabilities (belief)
#    probSensor  Grid of probability that (sensor==True)
#    sensor      Value of sensor
#    post        Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    # Create the posterior belief.
    post = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            if sensor:
                post[r, c] = probSensor[r, c] * prior[r, c]
            else:
                post[r, c] = (1 - probSensor[r, c]) * prior[r, c]

    
    # Normalize.
    s = np.sum(post)
    if (s == 0.0):
        print("LOST ALL BELIEF - THIS SHOULD NOT HAPPEN!!!!")
    else:
        post = (1.0/s) * post
    return post


#
#  Pre-compute the Sensor Probability Grid
#
#    drow, dcol    Direction in row/col
#    probProximal  List of probability that sensor triggers at dist=(index+1)
#    prob          Grid of probability that (sensor==True)
#
def precomputeSensorProbability(drow, dcol, probProximal = [1.0], kidnap=False):
    # Prepare an empty probability grid.
    prob = np.zeros((rows, cols))

    # Pre-compute the sensor probability on the grid.
    for r in range(rows):
        for c in range(cols):
            for i, prox in enumerate(probProximal):
                dist = i + 1
                
                new_r = r + dist * drow
                new_c = c + dist * dcol
                
                # If within grid and sensor sees walls at distance dist
                if 0 <= new_r < rows and 0 <= new_c < cols and walls[new_r, new_c] == 1:
                    prob[r, c] += prox
    
    # Ensures that no cell has zero belief by subtracting 5 percent of belief from top 10 cells with 
    # highest sensor probability
    if kidnap:
        percent = .05
        k = 10
        top_flat = np.argpartition(prob.flatten(), -k)[-k:]
        top_indices = np.array(np.unravel_index(top_flat[np.argsort(-prob.flatten()[top_flat])], prob.shape)).T
        total_nonzero_prob = 0
        for (r, c) in top_indices:
            total_nonzero_prob += prob[r, c] * percent
        nonzero_prob = total_nonzero_prob / np.count_nonzero(prob == 0)
        for r in range(rows):
            for c in range(cols):
                if prob[r, c] == 0: prob[r, c] += nonzero_prob

    # Return the computed grid.
    return prob

# 
#
#  Main Code
#
def main():
    # FIXME... PICK WHAT THE "REALITY" SHOULD SIMULATE:
    # Initialize the robot simulation.
    # robot=Robot(walls) # Prob 1(a)
    # robot=Robot(walls, row=start[0], col=start[1]) # Prob 1(b) !!!!!!6 is shortest path!!!!!!!!
    # Prob 2     robot=Robot(walls, row=12, col=26, probProximal=[0.9,0.6,0.3])
    # Prob 3     robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3],
    #                        probCmd=0.8)
    # Prob 4     robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3],
    #                        probCmd=0.8, kidnap=True)
    # Or to play robot=Robot(walls, probProximal=[0.9,0.6,0.3], probCmd=0.8)


    # Initialize your localization parameters.
    probCmd      = 1
    probProximal = [1]
    
    robot = Robot(walls, row=start[0], col=start[1], probProximal=probProximal, probCmd=probCmd)

    # Report.
    print("Localization is assuming probProximal = " + str(probProximal) +
          ", probCmd = " + str(probCmd))


    # Initialize the figure.
    visual = Visualization(walls, robot, goal)

    kidnap = False
    
    # Pre-compute the probability grids for each sensor reading.
    probUp    = precomputeSensorProbability(-1,  0, probProximal, kidnap=kidnap)
    probRight = precomputeSensorProbability( 0,  1, probProximal, kidnap=kidnap)
    probDown  = precomputeSensorProbability( 1,  0, probProximal, kidnap=kidnap)
    probLeft  = precomputeSensorProbability( 0, -1, probProximal, kidnap=kidnap)

    # Show the sensor probability maps.
    # visual.Show(probUp)
    # input("Probability of proximal sensor up reporting True")
    # visual.Show(probRight)
    # input("Probability of proximal sensor right reporting True")
    # visual.Show(probDown)
    # input("Probability of proximal sensor down reporting True")
    # visual.Show(probLeft)
    # input("Probability of proximal sensor left reporting True")


    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)


    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(bel, markRobot=True)

        # Get the command key to determine the direction.
        while True:
            key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
            if   (key == 'q'):  return
            elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
            elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
            elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
            elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break

        # Move the robot in the simulation.
        robot.Command(drow, dcol)


        # Compute a prediction.
        prd = computePrediction(bel, drow, dcol, probCmd)
        #visual.Show(prd)
        #input("Showing the prediction")

        # Check the prediction.
        if abs(np.sum(prd) - 1.0) > 1e-12:
            print("WARNING: Prediction does not add up to 100%")


        # Correct the prediction/execute the measurement update.
        bel = prd
        bel = updateBelief(bel, probUp,    robot.Sensor(-1,  0))
        bel = updateBelief(bel, probRight, robot.Sensor( 0,  1))
        bel = updateBelief(bel, probDown,  robot.Sensor( 1,  0))
        bel = updateBelief(bel, probLeft,  robot.Sensor( 0, -1))


if __name__== "__main__":
    main()
