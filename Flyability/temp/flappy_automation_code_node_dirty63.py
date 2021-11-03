#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

## Debug Help
DEBUG_SENSING = False
DEBUG_CONTROL = True

## Given game constants
SCALING = 0.01 # meter to pixel ratio [m/px]
FPS = 30 # game update loop frequency [Hz]

MAX_ACCX = 3  # max acceleration in x [m/s^2]
MAX_ACCY = 35 # max acceleration in y [m/s^2]


## Sensing settings
THRESH_DIST_ENV_EDGE = 2 # environment edge being the floor and ceiling

INTER_WALL_DIST = 2  # distance between two asteroid belts

## Other
EPSILON = 0.005 # float comparison precision

# Global data for information relay between velCallback and laserScanCallback
distance_floor = np.inf # [m]
distance_ceiling = np.inf # [m]
distance_wall = np.inf # [m]
distance_hole = np.inf # [m]

aligned_with_hole = False
search_hole_below = False
search_hole_above = False

# Utility functions
def distanceFloor(laser_ranges, angle_min, angle_delta):

    # compute distances
    dist0 = laser_ranges[0]*np.sin(angle_min)
    dist1 = laser_ranges[1]*np.sin(angle_min+angle_delta)
    # set distance to true value or infinity if non existent
    distance_floor = np.inf
    if np.abs(dist0 - dist1) < EPSILON:
	distance_floor = np.abs(dist0)

    if DEBUG_SENSING is True: print "distance to floor\t", distance_floor

    return distance_floor


def distanceCeiling(laser_ranges, angle_max, angle_delta):

    # compute distances
    dist0 = laser_ranges[-1]*np.sin(angle_max)
    dist1 = laser_ranges[-2]*np.sin(angle_max-angle_delta)
    # set distance to true value or infinity if non existent
    distance_ceiling = np.inf
    if np.abs(dist0 - dist1) < EPSILON:
	distance_ceiling = np.abs(dist0)

    if DEBUG_SENSING is True: print "distance to ceiling\t", distance_ceiling

    return distance_ceiling


def distanceWall(laser_ranges, angle_min, angle_max, angle_delta, \
		 distance_floor, distance_ceiling):

    # compute distances
    ranges = np.array(laser_ranges)
    angles = np.arange(angle_min, angle_max+angle_delta, angle_delta)

    distances_x = np.abs(ranges*np.cos(angles))    
    distances_y = np.abs(ranges*np.sin(angles))

    # discard laser beams hitting next wall interest or floor or ceiling
    distances_x[distances_x > INTER_WALL_DIST] = -1
    distances_x[np.abs(distances_y - distance_floor) < EPSILON] = -1
    distances_x[np.abs(distances_y - distance_ceiling) < EPSILON] = -1
    # average measures for distance estimation
    distance_wall = np.mean(distances_x[distances_x > 0])
    if np.isnan(distance_wall):
	distance_wall = np.inf

    if DEBUG_SENSING is True: print "distance to wall\t", distance_wall, \
			            "vs middle ray", laser_ranges[len(ranges)//2]

    return distance_wall


def findHole(laser_ranges, angle_min, angle_max, angle_delta):

    global search_below, search_above
    global aligned_with_hole
    global distance_wall, distance_hole

    ranges = np.array(laser_ranges)
    angles = np.arange(angle_min, angle_max+angle_delta, angle_delta)

    distances_x = np.abs(ranges*np.cos(angles)) 

    angle = np.inf 
    for i in range(len(laser_ranges)//2):
	if distances_x[i] > INTER_WALL_DIST and distances_x[i+1] > INTER_WALL_DIST:
	    search_below = True
	    search_above = False
	    angle = (angles[i] + angles[i+1])/2
	    break

	if distances_x[-i-1] > INTER_WALL_DIST and distances_x[-i-2] > INTER_WALL_DIST:
	    search_above = True
	    search_below = False
	    angle = (angles[-i-1] + angles[-i-2])/2
	    break    

    if angle == np.inf:
	if distances_x[0] > INTER_WALL_DIST:
	    search_below = True
	    angle = angles[0]
	elif distances_x[-1] > INTER_WALL_DIST:
	    search_above = True
	    angle = angles[-1]
	
	elif distances_x[1] > INTER_WALL_DIST:
	    search_below = True
	    angle = angles[1]
	elif distances_x[-2] > INTER_WALL_DIST:
	    search_above = True
	    angle = angles[-2]
	
    if not aligned_with_hole:
	if angle != np.inf:
    	    distance_hole = distance_wall*np.tan(angle)
        if np.isnan(distance_hole):
	    distance_hole = np.inf
    else:
	distance_hole = 0


    if DEBUG_SENSING is True: print "distance to hole", distance_hole, \
				    "BELOW" if search_below else \
				    "ABOVE" if search_above else \
				    "NOT FOUND"


def alignedWithHole(laser_ranges, laser_intensities, angle_min, angle_max, angle_delta):

    # select middle third of rays 
    third = len(laser_ranges)//3
    middle_ranges = laser_ranges
    middle_intensities = laser_intensities
    # compute distances
    ranges = np.array(middle_ranges)
    angles = np.arange(angle_min, angle_max+angle_delta, angle_delta)

    distances_x = np.abs(ranges*np.cos(angles))

    # flappy is aligned if all middle rays pass through wall
    aligned_with_hole = np.all(distances_x[third:-third] > 0.7*INTER_WALL_DIST)

    if DEBUG_SENSING is True: print "alignedWithHole", "YES" if aligned_with_hole else "NO"

    return aligned_with_hole


# Initialization and ROS callbacks
def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

## Control settings ## ============================================================= #
T_STOPX = 0.65 # desired time to stop in x [s]
T_STOPY = 0.15 # desired time to stop in y [s]
MAX_VELX = T_STOPX*MAX_ACCX # max speed in x to stop in T_STOPX [m/s] 
MAX_VELY = T_STOPY*MAX_ACCY # max speed in y to stop in T_STOPY [m/s]

errorXprior = 0
errorYprior = 0

THRESH_DIST_WALL_CTRLX = 2.0 # wall being the asteroid belt
THRESH_DIST_WALL_CTRLY = 2


def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    global distance_floor, distance_ceiling, distance_wall, distance_hole
    global aligned_with_hole 

    global search_above, search_below
    global errorXprior, errorYprior
    
    velx = msg.x
    vely = msg.y
    
    # align asap in X
    velx_d = 0    
    if distance_wall > THRESH_DIST_WALL_CTRLX or aligned_with_hole:
	velx_d = MAX_VELX#distance_wall/T_STOPX if velx < MAX_VELX else 0
	#accx =  KpX*(MAX_VELX - velx) if velx < MAX_VELX else 0
    else:
	#accx = -MAX_ACCX #-velx/T_STOPX # can't go negative
        velx_d = MAX_VELX*distance_wall 
    # look for hole in Y
    vely_d = 0

    if distance_wall < THRESH_DIST_WALL_CTRLY and not aligned_with_hole:
    	print distance_wall, distance_floor, distance_ceiling
	if search_below and distance_floor > THRESH_DIST_ENV_EDGE:
	    #accy = KpY*(-MAX_VELY - vely) if vely > -MAX_VELY else 0
	    #print "HOLE BELOW"
	    vely_d = distance_hole/T_STOPY#-MAX_VELY
	elif search_above and distance_ceiling > THRESH_DIST_ENV_EDGE:
	    #accy = KpY*(MAX_VELY - vely) if vely < MAX_VELY else 0
	    #print "HOLE ABOVE"
	    vely_d = distance_hole/T_STOPY#MAX_VELY
	else:    
	    if distance_floor > THRESH_DIST_ENV_EDGE: 
	    	#accy = KpY*(-MAX_VELY - vely) if vely > -MAX_VELY else 0
		#print "IDK GO DOWN"
		vely_d = -MAX_VELY
	    elif distance_ceiling > THRESH_DIST_ENV_EDGE:
	    	#accy = KpY*(MAX_VELY - vely) if vely < MAX_VELY else 0
		#print "IDK GO UP"
		vely_d = MAX_VELY
	      
	    else:
	    	#accy = -vely/T_STOPY
		#print "SHOULD NOT HAPPEN ---- RESET"
		vely_d = 0
		search_below = False
		search_above = False
    else:
	#"WAITING TO PASS HOLE"
	vely_d = 0
	search_below = False
	search_above = False

    # PD regulators u = Kp*error + Kd*(error - error_p)*sampling_freq
    KpX = 1
    #KdX = 1
    KpY =20 # peut monter bcp plus haut
    KdY = 0.

    errorX = velx_d - velx
    #derivativeX = (errorX - errorXprior)*FPS
    accx = KpX*errorX #+ KdX*derivativeX   
    errorXprior = errorX

    errorY = vely_d - vely
    derivativeY = (errorY - errorYprior)*FPS
    accy = KpY*errorY + KdY*derivativeY
    errorYprior = errorY

    if np.isnan(accy): accy = 0

    if DEBUG_CONTROL: print "accx", accx, "velx", velx, "accy", accy, "vely", vely
    #return
    # The following saturates at ACCXLIMIT, ACCYLIMIT by itself
    pub_acc_cmd.publish(Vector3(accx,accy,0))


def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    global distance_floor, distance_ceiling, distance_wall
    global aligned_with_hole 

    msg_ranges_list = list(msg.ranges)

    distance_floor = distanceFloor(msg_ranges_list, msg.angle_min, msg.angle_increment)
    distance_ceiling = distanceCeiling(msg_ranges_list, msg.angle_max, msg.angle_increment)
    distance_wall = distanceWall(msg_ranges_list, msg.angle_min, msg.angle_max, \
				 msg.angle_increment, distance_floor, distance_ceiling)
    aligned_with_hole = alignedWithHole(msg_ranges_list, msg.intensities, msg.angle_min, msg.angle_max, msg.angle_increment)

    findHole(msg_ranges_list, msg.angle_min, msg.angle_max, msg.angle_increment)


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
