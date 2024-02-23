import numpy as np
import math
import Constants.configDrones as configDrones

EVIDENCE = configDrones.EVIDENCE
BOTTOM_LEFT_BOUND = configDrones.BOTTOM_LEFT_BOUND
TOP_RIGHT_BOUND = configDrones.TOP_RIGHT_BOUND
grid_index_width = configDrones.grid_index_width
grid_index_height = configDrones.grid_index_height
Grid = None

def startBayes(size):
    #print(gridIndexWidth, "  ", gridIndexHeight)
    Grid = initializeGrid(size)
    return Grid

def initializeGrid(size):
    """ Initialize a uniform probability grid. """
    grid = np.full((size[0], size[1]), 1/(size[0]*size[1]))  # Uniform distribution (All grid values summed == 1)
    return grid

def find_max_probability_cell(grid):
    """ Find the index of the cell with the highest probability. """
    return np.unravel_index(np.argmax(grid), grid.shape)

def apply_evidence_to_cell(GPSCoord): 
    """ Apply evidence to a specific cell. """
    # cell = (row, column)
    cell = gps_to_grid(GPSCoord[0], GPSCoord[1])
    print("Cell:", cell)
    likelihood = np.ones(Grid.shape)
    likelihood[cell] = EVIDENCE
    return update_grid_with_evidence(likelihood)

def update_grid_with_evidence(Grid, evidence_grid):
    """ Update the grid probabilities based on new evidence using Bayes' Theorem. """
    new_grid = Grid * evidence_grid
    Grid = new_grid / new_grid.sum()
    print("Grid: ", Grid)
    return Grid

def gps_to_grid(latitude, longitude):
    """ Convert the GPS coordinate to the appropriate location in the Bayes grid"""
    print(((grid_index_width)))
    xIndex = math.floor(((longitude - BOTTOM_LEFT_BOUND[1]) / grid_index_height))       # cell = (row, column) so this may seem reversed 
    yIndex = math.floor(((latitude - BOTTOM_LEFT_BOUND[0]) / grid_index_width))   
    return (xIndex, yIndex)

if __name__ == '__main__':
    size = (10, 20) # (y,x)
    #startBayes(size, (0.00014477851375283054,-0.0041943628509558415), (0.004516586943213801, 0.004839834080485371))
    startBayes(size)
    GPSCoord = (0.0003863109910887481,0.0016887219677169038)
    #print(grid)
    gridvalue = gps_to_grid(GPSCoord[0], GPSCoord[1])
    print(gridvalue)
    print('test')