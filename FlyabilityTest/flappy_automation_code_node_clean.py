#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

""" VERSION Best Score 63 ================================================== """

## Debug Help
DEBUG_SENSING = False
DEBUG_CONTROL = False
HUMAN_CONTROL = False

## Given game constants
SCALING = 0.01 # meter to pixel ratio [m/px]
FPS = 30 # game update loop frequency [Hz]

MAX_ACCX = 3  # max acceleration in x [m/s^2]
MAX_ACCY = 35 # max acceleration in y [m/s^2]

## Other
EPSILON = 0.005 # float comparison precision

## Global data for information relay between velCallback and laserScanCallback
distance_floor = np.inf # [m]
distance_ceiling = np.inf # [m]
distance_wall = np.inf # [m]
distance_hole = np.inf # [m]

aligned_with_hole = False
search_below = False
search_above = False

""" SENSING ================================================================ """

## Sensing settings
INTER_WALL_DIST = 2  # distance between two asteroid belts

## Commonly used parameters related to lasers, global to avoid recomputing
ranges = None
angles = None
distances_x = None
distances_y = None

def computeCommonParameters(laser_ranges, angle_min, angle_max, angle_delta):
    """ Compute parameters used by sensing functions
	global ranges: laser ray distance to contact [m], max value if no contact
 	global angles: laser ray orientation angles [rad]
	global distances_x: range projection onto x-axis [m] (horizontal)
	global distances_x: range projection onto y-axis [m] (vertical)

	laser_ranges: tuple of ranges as given by LaserScan msg [m]
	angle_min: angle of the first laser range as given by LaserScan msg [rad] (-)
	angle_max: angle of the last  laser range as given by LaserScan msg [rad] (+)
	angle_delta: constant angle increment as given by LaserScan msg [rad]
    """
    global ranges, angles, distances_x, distances_y

    ranges = np.array(laser_ranges)
    angles = np.arange(angle_min, angle_max+angle_delta, angle_delta)
    distances_x = np.abs(ranges*np.cos(angles))    
    distances_y = np.abs(ranges*np.sin(angles))


def distanceFloor():
    """ Determine distance to the floor and assign it to global distance_floor
	note: defaults to np.inf if ceiling not detected 
	note: use of first two rays and EPSILON is calibrated to the default 9 laser rays
    """
    global distances_y

    global distance_floor

    # set distance to true value or infinity if non existent
    distance_floor = np.inf
    if np.abs(distances_y[0] - distances_y[1]) < EPSILON:
	distance_floor = np.abs(distances_y[0])

    if DEBUG_SENSING is True: print "distance to floor\t", distance_floor


def distanceCeiling():
    """ Determine distance to the ceiling and assign it to global distance_ceiling 
	note: defaults to np.inf if ceiling not detected
	note: use of last two rays and EPSILON is calibrated to the default 9 laser rays
    """
    global distances_y

    global distance_ceiling

    # set distance to true value or infinity if non existent
    distance_ceiling = np.inf
    if np.abs(distances_y[-1] - distances_y[-2]) < EPSILON:
	distance_ceiling = np.abs(distances_y[-1])

    if DEBUG_SENSING is True: print "distance to ceiling\t", distance_ceiling


def distanceWall():
    """ Determine distance to the wall and assign it to global distance_wall 
	note: defaults to np.inf if measure incoherent
	note: use of last two rays and EPSILON is calibrated to the default 9 laser rays
    """
    global distances_x, distances_y, distance_floor, distance_ceiling

    global distance_wall

    # copy array to avoid modifying global variable
    select_distances_x = np.copy(distances_x)
    # disregard inappropriate laser ranges bouncing off unwanted features
    select_distances_x[select_distances_x > INTER_WALL_DIST] = -1
    select_distances_x[np.abs(distances_y - distance_floor) < EPSILON] = -1
    select_distances_x[np.abs(distances_y - distance_ceiling) < EPSILON] = -1

    distance_wall = np.mean(distances_x[select_distances_x > 0])

    # compensate for numerical inconsistencies
    if np.isnan(distance_wall):
	distance_wall = np.inf

    if DEBUG_SENSING is True: print "distance to wall\t", distance_wall


def distanceHole():
    """ Determine distance to the hole and assign it to global distance_hole
	note: defaults to np.inf if measure incoherent

	This function sets flags for the controller
	global aligned_with_hole: aligned with a hole
	global search_below: seems like the hole is below 
	global search_above: seems like the hole is above 

	note: this is calibrated to the default 9 laser rays
    """
    global distances_x, distance_wall 

    global distance_hole, aligned_with_hole, search_below, search_above

    third = len(distances_x)//3
    # check middle rays, if they traverse wall either aligned with hole or bound to crash
    if np.all(distances_x[third:-third] > 0.7*INTER_WALL_DIST):
	aligned_with_hole = True
	search_below = False
	search_above = False
	distance_hole = 0
    else:
	aligned_with_hole = False
    	angle = np.inf 
	# look for two consecutive rays traversing wall (very likely to be the hole)
    	for i in range(len(distances_x)//2):
	    if distances_x[i] > INTER_WALL_DIST and distances_x[i+1] > INTER_WALL_DIST:
	    	angle = (angles[i] + angles[i+1])/2
	    	break

	    if distances_x[-i-1] > INTER_WALL_DIST and distances_x[-i-2] > INTER_WALL_DIST:
	    	angle = (angles[-i-1] + angles[-i-2])/2
	    	break    

        if angle == np.inf:
	    # unsuccessful, see if extreme rays traverse the wall (quite likely to be the hole)
	    if distances_x[0] > INTER_WALL_DIST:
	    	angle = angles[0]
	    elif distances_x[-1] > INTER_WALL_DIST:
	    	angle = angles[-1]
	    # see if less extreme rays traverse the wall (less likely to be the hole)
	    elif distances_x[1] > INTER_WALL_DIST:
	    	angle = angles[1]
	    elif distances_x[-2] > INTER_WALL_DIST:
	    	angle = angles[-2]
	
	if angle != np.inf:
    	    distance_hole = np.abs(distance_wall*np.tan(angle))
	    if angle > 0:
	    	search_above = True
		search_below = False
	    else:
        	search_below = True
		search_above = False

	# compensate for numerical inconsistencies
	if np.isnan(distance_hole):
	    distance_hole = np.inf

    if DEBUG_SENSING is True: print "distance to hole", distance_hole, \
				    "BELOW" if search_below else \
				    "ABOVE" if search_above else \
				    "NOT FOUND"


def laserScanCallback(msg):
    """ ROS Callback with msg containing laser measures """

    # msg has the format of sensor_msgs::LaserScan
    computeCommonParameters(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)

    distanceFloor() 
    distanceCeiling()
    distanceWall() 
    distanceHole()


""" CONTROL ================================================================ """

## Control settings 
SAFETY_DIST_ENV_EDGE = 2 # distance to maintain from environment edge 
			 # (inflated to take into account deceleration time)

T_STOPX = 0.65 # desired time to stop in x [s]
T_STOPY = 0.15 # desired time to stop in y [s]
MAX_VELX = T_STOPX*MAX_ACCX # max speed in x to stop in T_STOPX [m/s] 
MAX_VELY = T_STOPY*MAX_ACCY # max speed in y to stop in T_STOPY [m/s]
KpX = 40
KpY = 40

def velCallback(msg):
    """ ROS Callback with msg containing velocity measures """
    global distance_floor, distance_ceiling, distance_wall, distance_hole
    global aligned_with_hole, search_above, search_below

    # msg has the format of geometry_msgs::Vector3
    velx = msg.x
    vely = msg.y
    
    # x-axis control
    velx_d = 0    
    if aligned_with_hole:
	# go as fast as possible
	velx_d = MAX_VELX
    else:
	# slow down when not aligned with hole
        velx_d = MAX_VELX*distance_wall 
    
    # y-axis control
    vely_d = 0
    if aligned_with_hole:
	# wait to pass hole
	vely_d = 0
    else:
	# align with hole
	if search_below and distance_floor > SAFETY_DIST_ENV_EDGE:
	    vely_d = -distance_hole/T_STOPY
	elif search_above and distance_ceiling > SAFETY_DIST_ENV_EDGE:
	    vely_d = distance_hole/T_STOPY
	else:    
	    # pick a safe direction at random until path is found
	    if distance_floor > SAFETY_DIST_ENV_EDGE:
		vely_d = -0.2*MAX_VELY
	    elif distance_ceiling > SAFETY_DIST_ENV_EDGE:
		vely_d = 0.2*MAX_VELY
	    else: 
		# should never happen, acts as reset
		vely_d = 0

    # proportional regulator based on speed error
    accx = KpX*(velx_d - velx)
    accy = KpY*(vely_d - vely)

    # compensate for numerical inconsistencies
    if np.isnan(accx): accx = 0
    if np.isnan(accy): accy = 0

    if DEBUG_CONTROL: print "accx", accx, "velx", velx, "accy", accy, "vely", vely
    
    if HUMAN_CONTROL: return

    # The following saturates acceleration by itself
    pub_acc_cmd.publish(Vector3(accx,accy,0))


""" ======================================================================== """

# Initialization and ROS callbacks
def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
