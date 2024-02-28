import numpy as np
import math
import Constants.configDrones as configDrones
import matplotlib.pyplot as plt
import json
import os

EVIDENCE = configDrones.EVIDENCE
BOTTOM_LEFT_BOUND = configDrones.BOTTOM_LEFT_BOUND
TOP_RIGHT_BOUND = configDrones.TOP_RIGHT_BOUND
grid_index_width = configDrones.grid_index_width
grid_index_height = configDrones.grid_index_height
global Grid

class BayesGrid:
    Grid = None

    def __init__(self, size):
        if BayesGrid.Grid is None:
            print("Initializing Grid")
            BayesGrid.Grid = self.initializeGrid(size)

    def initializeGrid(self, size):
        """ Initialize a uniform probability grid. """
        grid = np.full((size[0], size[1]), 1/(size[0]*size[1]))  # Uniform distribution (All grid values summed == 1)
        return grid

    def find_max_probability_cell(self):
        """ Find the index of the cell with the highest probability. """
        return np.unravel_index(np.argmax(self.Grid), self.Grid.shape)

    def apply_evidence_to_cell(self, GPSCoord):
        """ Apply evidence to a specific cell. """
        # cell = (row, column)
        if(self.is_in_bounds(GPSCoord[0], GPSCoord[1])):
            cell = self.gps_to_grid(GPSCoord[0], GPSCoord[1])
            print("Cell (y,x):", cell)
            likelihood = np.ones(self.Grid.shape)
            likelihood[cell] = EVIDENCE
            self.update_grid_with_evidence(likelihood)
        else:
            print("GPS Coordinate Out of Bounds")

    def update_grid_with_evidence(self, evidence_grid):
        """ Update the grid probabilities based on new evidence using Bayes' Theorem. """
        new_grid = self.Grid * evidence_grid
        self.Grid = new_grid / new_grid.sum()
        #print("Grid: ", self.Grid)

    def gps_to_grid(self, latitude, longitude):
        """ Convert the GPS coordinate to the appropriate location in the Bayes grid"""
        print(((grid_index_width)))

        xIndex = math.floor(((longitude - BOTTOM_LEFT_BOUND[1]) / grid_index_width))       # cell = (row, column) so this may seem reversed 
        yIndex = math.floor(((latitude - BOTTOM_LEFT_BOUND[0]) / grid_index_height))   
        return (yIndex, xIndex)
    
    def is_in_bounds(self, latitude, longitude):
        """ Check if the GPS coordinate is within the defined bounds. """
        if (BOTTOM_LEFT_BOUND[0] <= latitude <= TOP_RIGHT_BOUND[0]) and \
        (BOTTOM_LEFT_BOUND[1] <= longitude <= TOP_RIGHT_BOUND[1]):
            return True
        else:
            return False

    def save_to_file(filename="grid_state.json"):
        if BayesGrid.Grid is not None:
            # Construct the path to the Constants folder dynamically
            current_dir = os.path.dirname(__file__)
            constants_dir = os.path.join(current_dir, "Constants")
            # Ensure the Constants directory exists
            os.makedirs(constants_dir, exist_ok=True)
            file_path = os.path.join(constants_dir, filename)

            with open(file_path, 'w') as file:
                json.dump(BayesGrid.Grid.tolist(), file)
        else:
            print("Grid is not initialized.")

    def load_from_file(filename="grid_state.json"):
        # Construct the path to the Constants folder dynamically
        current_dir = os.path.dirname(__file__)
        constants_dir = os.path.join(current_dir, "Constants")
        file_path = os.path.join(constants_dir, filename)

        try:
            with open(file_path, 'r') as file:
                grid_list = json.load(file)
                BayesGrid.Grid = np.array(grid_list)
        except FileNotFoundError:
            print(f"File {filename} not found in {constants_dir}.")


if __name__ == '__main__':
    size = (10, 20) # (y,x)
    #startBayes(size, (0.00014477851375283054,-0.0041943628509558415), (0.004516586943213801, 0.004839834080485371))
    bayes_grid = BayesGrid(size)
    GPSCoord = (0.0024477851375283054,-0.0021943628509558415) # Must be represented as (latitude, longitude)
    BayesGrid.load_from_file()
    print(bayes_grid.Grid)
    
    # gridvalue = bayes_grid.gps_to_grid(GPSCoord[0], GPSCoord[1])
    # print(gridvalue)
    bayes_grid.apply_evidence_to_cell(GPSCoord)
    print(bayes_grid.Grid)
    bayes_grid.apply_evidence_to_cell(GPSCoord)
    print(bayes_grid.Grid)
    bayes_grid.apply_evidence_to_cell(GPSCoord)
    print(bayes_grid.Grid)