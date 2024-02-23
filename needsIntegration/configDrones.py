# shared non export
min_circle_radius_gps = 0.00008983152373552244
# shared end

LOOP_NUMBER = 100000
MAX_TIME = 1200
LOCAL_IP = "172.28.160.1"
# collision start
COLLISION_MODE_TIME_LENGTH = 1
MAX_COLLISION_TIME = 3
MIN_COLLISION_TIME = 0.25
# collision end
# vector control start
MAX_TURN_ANGLE = 30
SPEED_CHANGE = 0.1
MIN_SPEED_FACTOR = 0.4
# vector control end
# linebehavior start
DISTANCE_LEAD_OVERSEER_GPS = min_circle_radius_gps * 2
AVOID_FACTOR = 0.01
DIRECTION_FACTOR = 5
WOLF_SLOW_DOWN = 3
OVERSEER_DIRECTION_SPEED_UP = 16
OVERSEER_DIRECTION_SLOW_DOWN = 5
OVERSEER_TO_WOLF_GROUP_RADIUS = 0.0003
REPULSION_RADIUS = 0.0003
# waypoint 
MAX_WAYPOINT_SAVE_TIME = 100    # measured in seconds
WAYPOINT_HISTORY_DISTANCE_MULT = 1
WAPOINT_HISTORY_DISTANCE_ERROR = 0.0001
# linebehavior end
# circling start
MIN_CIRCLE_RADIUS_GPS = min_circle_radius_gps # 10 in x direction converted to gps
MIN_CIRCLE_RADIUS_METERS = 6.988048291572515 # 10 in x direction converted to Meters
MIN_DIFFRENCE_IN_RADIUS = min_circle_radius_gps * 0.2
REQUIRED_SEPERATION_PERCENT = 0.8
MIN_CIRCLE_PADDING_FOR_SEARCH_HISTORY = min_circle_radius_gps * 1.1
# circling end
# wolf search start
WOLF_SEARCH_REQUEST_HELP_DISTANCE_MULTIPLE = 1.5
CIRCLE_SPACING = min_circle_radius_gps * 0.2
# MIN_CIRCLE_RADIUS_GPS
# MIN_CIRCLE_RADIUS_METERS
# wolf search end
# consenus start
CONSENSUS_DECISION_REQUEST_HELP_DISTANCE_MULTIPLE = 3
MAX_CONSENSUS_ITERATION_NUMBER = 3
CONSENSUS_ITERATION_LENGTH_SECONDS = 15
CONSENSUS_THRESHOLD = 0.2
YOLO_CONFIDENCE = 0.5
DETECTION_OUTPUT_SERIES = 'mapFinal05'
ZOOM_FACTOR = 2000000
MAX_PIX_COUNT = 4500
SPIRAL_LOCATION_1 = [0, 0.0011228941075]
#SPIRAL_LOCATION_1 = [0.0011228941075, 0]
# MIN_CIRCLE_RADIUS_GPS
# MIN_CIRCLE_RADIUS_METERS
# consenus end
# Bayes Start
GRID_SIZE = (10,20)
BOTTOM_LEFT_BOUND = (0.00014477851375283054,-0.0041943628509558415)
TOP_RIGHT_BOUND = (0.004516586943213801, 0.004839834080485371)
EVIDENCE = 0.7 # This value is changeable. It is the probability that if we search a cell we should find the target if the target is there (30 percent error currently)
grid_width = TOP_RIGHT_BOUND[0] - BOTTOM_LEFT_BOUND[0]
grid_height = TOP_RIGHT_BOUND[1] - BOTTOM_LEFT_BOUND[1]
# this is the width and height of each index/grid space
grid_index_width = grid_width/GRID_SIZE[1] 
grid_index_height = grid_height/GRID_SIZE[0]
# Bayes End