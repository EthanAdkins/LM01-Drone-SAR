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
THRESHOLD_LAT = configDrones.THRESHOLD_LAT  # Example threshold for latitude
THRESHOLD_LON = configDrones.THRESHOLD_LON  # Example threshold for longitude

class BayesGrid:
    Grid = None
    
    def __init__(self, size):
        self.seen_search_ids = set()
        if BayesGrid.Grid is None:
            print("Initializing Grid")
            BayesGrid.Grid = self.initialize_grid_prior(size)
            #BayesGrid.save_to_file()

    def initializeGrid(self, size):
        """ Initialize a uniform probability grid. """
        grid = np.full((size[0], size[1]), 1/(size[0]*size[1]))  # Uniform distribution (All grid values summed == 1)
        return grid

    def initialize_clustered_grid(self, size, num_clusters=3, spread=3):
        """ Initialize a probability grid with clustered high probability areas. """
        grid = np.zeros((size[0], size[1]))

        # Randomly choose cluster centers
        cluster_centers = [np.random.randint(0, min(size[0], size[1]), size=2) for _ in range(num_clusters)]

        for i in range(size[0]):
            for j in range(size[1]):
                # Calculate probability based on distance to nearest cluster center
                min_distance = min([np.linalg.norm(np.array([i, j]) - center) for center in cluster_centers])
                grid[i, j] = np.exp(-min_distance / spread)

        # Normalize the grid so probabilities sum to 1
        grid /= grid.sum()
        return grid
    
    def initialize_grid_prior(self, size):
        """ Initialize a probability grid with clustered high probability areas. """
        BayesGrid.Grid = self.initializeGrid(size)
        #self.increase_probability_by_gps([0.000021918283978328166,0])
        #self.increase_probability_by_gps([0.003, 0.002])
        #self.increase_probability_by_gps([0.0026853068927250876, -0.00018847169112613604])
        self.increase_probability_by_gps([0.0010895132618108832, -0.0045924587249556925])
        return BayesGrid.Grid

    def find_max_probability_cell(self, significance_threshold=-1):
        """
        Find the most probable location in the grid and its probability.
        Only returns a location if the highest probability exceeds the second highest by the given threshold.

        Parameters:
        - significance_threshold: The minimum difference between the highest and second-highest probabilities required to consider a cell significantly most probable.
        """
        # Set significance_threshold to -1 if you want no threshold

        # Flatten the grid to make it easier to work with
        flat_grid = BayesGrid.Grid.flatten()
        
        # Sort the flattened grid to find the highest and second-highest probabilities
        sorted_probs = np.sort(flat_grid)
        highest_prob = sorted_probs[-1]
        second_highest_prob = sorted_probs[-2]
        
        # Check if the highest probability is significantly greater than the second highest
        if highest_prob - second_highest_prob > significance_threshold:
            # Find the index of the cell with the highest probability
            max_prob_index = np.unravel_index(np.argmax(BayesGrid.Grid), BayesGrid.Grid.shape)
            #print(f"Most Probable Location: {max_prob_index}, Probability: {highest_prob}")
            return max_prob_index, highest_prob
        else:
            # print("No significantly most probable location found.")
            return None, None

    def apply_evidence_to_cell(self, GPSCoord, searchID):
        """ Apply evidence to a specific cell. """
        # Returns True if updated, False if not
        
        # Check if searchID has been seen before
        if searchID in self.seen_search_ids:
            print(f"Search ID {searchID} has been seen before.")
            return False, None
        
        # If searchID is new, add it to the set
        self.seen_search_ids.add(searchID)
        print("Added: ", searchID," to seen Search IDs")

        # cell = (row, column)
        if(self.is_in_bounds(GPSCoord[0], GPSCoord[1])):
            cell = self.gps_to_grid(GPSCoord[0], GPSCoord[1])
            #print("Cell (y,x):", cell)
            likelihood = np.ones(BayesGrid.Grid.shape)
            likelihood[cell] = EVIDENCE
            self.update_grid_with_evidence(likelihood)
            #BayesGrid.save_to_file()
            gridString = self.save_to_string()
            return True, gridString
        else:
            print("GPS Coordinate: ", GPSCoord, " Out of Bounds")
            return False, None

    def update_grid_with_evidence(self, evidence_grid):
        """ Update the grid probabilities based on new evidence using Bayes' Theorem. """
        new_grid = BayesGrid.Grid * evidence_grid
        BayesGrid.Grid = new_grid / new_grid.sum()
        #print("Grid: ", self.Grid)

    def gps_to_grid(self, latitude, longitude):
        """ Convert the GPS coordinate to the appropriate location in the Bayes grid"""
        # print(((grid_index_width)))

        latitude = max(BOTTOM_LEFT_BOUND[0], min(TOP_RIGHT_BOUND[0], latitude))
        longitude = max(BOTTOM_LEFT_BOUND[1], min(TOP_RIGHT_BOUND[1], longitude))

        xIndex = math.floor((longitude - BOTTOM_LEFT_BOUND[1]) / grid_index_width)
        yIndex = math.floor((latitude - BOTTOM_LEFT_BOUND[0]) / grid_index_height)
        return (yIndex, xIndex)
    
    def is_in_bounds(self, latitude, longitude):
        """Check if the GPS coordinate is within the defined bounds or slightly outside within a threshold."""
        # Check latitude within bounds considering the threshold
        in_lat_bounds = (BOTTOM_LEFT_BOUND[0] - THRESHOLD_LAT <= latitude <= TOP_RIGHT_BOUND[0] + THRESHOLD_LAT)
        
        # Check longitude within bounds considering the threshold
        in_lon_bounds = (BOTTOM_LEFT_BOUND[1] - THRESHOLD_LON <= longitude <= TOP_RIGHT_BOUND[1] + THRESHOLD_LON)
        
        return in_lat_bounds and in_lon_bounds
        
    def centerGrid_to_GPS(self, Index):
        """ Convert an index of the Bayes grid to the GPS center of that grid index"""
        yIndex = Index[0] 
        xIndex = Index[1]
        latitude = yIndex * grid_index_height + (grid_index_height / 2.0) + BOTTOM_LEFT_BOUND[0]
        longitude = xIndex * grid_index_width + (grid_index_width / 2.0) + BOTTOM_LEFT_BOUND[1]
        #print("LatitudeConvert: ", latitude, "LongitudeConvert: ", longitude)
        return (latitude, longitude)


    # def save_to_file(filename="grid_state.json"):
    #     print("SAVING")
    #     if BayesGrid.Grid is not None:
    #         # Construct the path to the Constants folder dynamically
    #         current_dir = os.path.dirname(__file__)
    #         constants_dir = os.path.join(current_dir, "Constants")
    #         # Ensure the Constants directory exists
    #         os.makedirs(constants_dir, exist_ok=True)
    #         file_path = os.path.join(constants_dir, filename)

    #         with open(file_path, 'w') as file:
    #             json.dump(BayesGrid.Grid.tolist(), file)
    #     else:
    #         print("Grid is not initialized.")

    # def load_from_file(filename="grid_state.json"):
    #     # Construct the path to the Constants folder dynamically
    #     current_dir = os.path.dirname(__file__)
    #     constants_dir = os.path.join(current_dir, "Constants")
    #     file_path = os.path.join(constants_dir, filename)

    #     try:
    #         with open(file_path, 'r') as file:
    #             grid_list = json.load(file)
    #             BayesGrid.Grid = np.array(grid_list)
    #     except FileNotFoundError:
    #         print(f"File {filename} not found in {constants_dir}.")
    def save_to_string(self):
        if BayesGrid.Grid is not None:
            # Convert numpy array to a list and serialize to JSON string
            json_string = json.dumps(BayesGrid.Grid.tolist())
            return json_string
        else:
            print("Grid is not initialized.")
            return None
        
    @staticmethod
    def load_from_string(json_string):
        # Deserialize JSON string back to a list
        grid_list = json.loads(json_string)
        
        # Convert list back to a numpy array and assign it to BayesGrid.Grid
        BayesGrid.Grid = np.array(grid_list)

        print("Loaded Grid")

    def increase_probability_by_gps(self, GPSCoord, factor=1.5, neighbor_factor=1.01):
        """
        Increases the probability of a specific cell determined by GPS coordinates
        and slightly increases the probability of the neighboring cells.

        Parameters:
        - GPSCoord: Tuple containing the latitude and longitude of the evidence.
        - factor: The factor by which to increase the probability in the specified cell.
        - neighbor_factor: The factor by which to increase the probabilities of the neighboring cells.
        """
        if not self.is_in_bounds(GPSCoord[0], GPSCoord[1]):
            print("GPS Coordinate: ", GPSCoord, " Out of Bounds")
            return False, None

        cell = self.gps_to_grid(GPSCoord[0], GPSCoord[1])
        print("Increasing probability for Cell (y,x):", cell)

        # Apply the factor to the target cell
        BayesGrid.Grid[cell] *= factor

        # Get the size of the grid to prevent index out of bounds
        max_y, max_x = BayesGrid.Grid.shape

        # Define the range of neighboring cells to adjust
        neighbors = [(i, j) for i in range(-1, 2) for j in range(-1, 2) if not (i == 0 and j == 0)]
        
        # Adjust the probabilities for the neighboring cells
        for dy, dx in neighbors:
            new_y, new_x = cell[0] + dy, cell[1] + dx
            if 0 <= new_y < max_y and 0 <= new_x < max_x:  # Check if within bounds
                BayesGrid.Grid[new_y, new_x] *= neighbor_factor

        # Ensure the probabilities sum to 1 after modification
        BayesGrid.Grid /= BayesGrid.Grid.sum()

        print("Probability increased for cell and neighbors:", cell)
        gridString = self.save_to_string()
        return True, gridString




if __name__ == '__main__':
    size = (10, 20) # (y,x)
    #startBayes(size, (0.00014477851375283054,-0.0041943628509558415), (0.004516586943213801, 0.004839834080485371))
    bayes_grid = BayesGrid(size)
    GPSCoord = (0.00036336893522587903, -0.003968507927669811) # Must be represented as (latitude, longitude)
    print(bayes_grid.Grid)
    
    gridvalue = bayes_grid.gps_to_grid(GPSCoord[0], GPSCoord[1])
    print("GRID VALUE: ", gridvalue)
    bayes_grid.apply_evidence_to_cell(GPSCoord,1)
    print(bayes_grid.Grid)
    bayes_grid.apply_evidence_to_cell(GPSCoord,2)
    print(bayes_grid.Grid)
    bayes_grid.apply_evidence_to_cell(GPSCoord,3)
    print(bayes_grid.Grid)
    max_prob_index, max_prob_value = bayes_grid.find_max_probability_cell(significance_threshold=-1)
    print("Index of Max: ", max_prob_index, " Value of Max: ", max_prob_value)
    maxGPS = bayes_grid.centerGrid_to_GPS(max_prob_index)
    print("GPS Coord of Max: ", maxGPS)