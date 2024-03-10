import Constants.configDrones as configDrones
from BayesTheorem import BayesGrid 

SPIRAL_LOCATION_1 = configDrones.SPIRAL_LOCATION_1
TOP_RIGHT_BOUND = configDrones.TOP_RIGHT_BOUND
BOTTOM_LEFT_BOUND = configDrones.BOTTOM_LEFT_BOUND
GRID_SIZE = configDrones.GRID_SIZE
grid_index_width = configDrones.grid_index_width
grid_index_height = configDrones.grid_index_height
global bayes_grid 
bayes_grid = BayesGrid(GRID_SIZE)

def is_within_bounds(coordinate):
    """Check if the given coordinate is within the grid bounds."""
    return (BOTTOM_LEFT_BOUND[0] <= coordinate[1] <= TOP_RIGHT_BOUND[0]) and \
           (BOTTOM_LEFT_BOUND[1] <= coordinate[0] <= TOP_RIGHT_BOUND[1])

def spiralSearchCoordinateMaker(groupName, spawnLocation, startLocation, amountOfWaypoints):
    global bayes_grid
    waypointsList = [None]*amountOfWaypoints        # List creation for waypoints
    currentWaypoint = startLocation                 # Sets current waypoint as our start location

    with open(groupName, 'w') as f:
        direction = 0 # Direction show via int, 1 = upward, 2 = right, 3 = downward, 4 = left
        currentWaypointText = str(currentWaypoint[0]) + " " + str(currentWaypoint[1]) + "\n"
        spawnPointText = str(spawnLocation[0]) + " " + str(spawnLocation[1]) + "\n"
        f.write(spawnPointText)
        f.write(currentWaypointText)

        # Used to change starting direction
        direction = 4  # Start by moving left

        for generalWaypoints in range(int(amountOfWaypoints)):
            waypointCount = generalWaypoints + 1

            for num in range(2):  # Turn after completing two sides of the current spiral loop
                direction = (direction % 4) + 1  # Ensure direction cycles through 1-4

                for _ in range(waypointCount):
                    if direction == 1 and is_within_bounds([currentWaypoint[0], currentWaypoint[1] + grid_index_height]):  # Up
                        currentWaypoint[1] += grid_index_height
                        #print("MOVING UP")
                    elif direction == 2 and is_within_bounds([currentWaypoint[0] + grid_index_width, currentWaypoint[1]]):  # Right
                        currentWaypoint[0] += grid_index_width
                        #print("MOVING RIGHT")
                    elif direction == 3 and is_within_bounds([currentWaypoint[0], currentWaypoint[1] - grid_index_height]):  # Down
                        currentWaypoint[1] -= grid_index_height
                        #print("MOVING DOWN")
                    elif direction == 4 and is_within_bounds([currentWaypoint[0] - grid_index_width, currentWaypoint[1]]):  # Left
                        currentWaypoint[0] -= grid_index_width
                        #print("MOVING LEFT")
                    else:
                        # Skip this movement if it would go out of bounds
                        continue
                    
                # Writes output to file
                outputString = str(currentWaypoint[0]) + " " + str(currentWaypoint[1])
                f.write(outputString)
                f.write('\n')

    f.close()

def createWaypoints():
    # Creates waypoints for group 0 to move to
    waypointDistance = 0.0003
    spawnLocation = [0.0001, 0.0001]
    center_latitude = (BOTTOM_LEFT_BOUND[0] + TOP_RIGHT_BOUND[0]) / 2
    center_longitude = (BOTTOM_LEFT_BOUND[1] + TOP_RIGHT_BOUND[1]) / 2
    # yIndex, xIndex = bayes_grid.gps_to_grid(center_latitude, center_longitude)
    # print("yIndex: ", yIndex, "xIndex: ", xIndex)
    centerStartLocation = [center_longitude,center_latitude]
    amountOfWaypoints = (GRID_SIZE[0] * GRID_SIZE[1])
    spiral0Filename = 'Constants/Group0Spiral.txt'
    spiralSearchCoordinateMaker(spiral0Filename, spawnLocation, centerStartLocation, amountOfWaypoints)

    # Creates waypoints for group 1 to move to
    waypointDistance = 0.0004                                   # Distance between waypoints
    spawnLocation = [-0.0001, 0.0001]
    centerStartLocation = [-0.0011228941075, 0.0011228941075]                     # Starting center of the spiral
    amountOfWaypoints =  5                                       # Amount of edges
    spiral1Filename = 'Constants/Group1Spiral.txt'
    spiralSearchCoordinateMaker(spiral1Filename, spawnLocation, centerStartLocation, amountOfWaypoints)
