# -*- coding: utf-8 -*-
"""
Created on Fri May 18 16:22:45 2018
PYTHON 2
list of functions interacting with dronekit, accepting strings as input to execute functions
@author: OliG
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, argparse


# sitl stuff   ###########################
import dronekit_sitl
sitl = dronekit_sitl.start_default()
##########################################

global drone_position_notify
drone_position_notify = [0,0]

def connection():
    # literally no clue what these 5 lines do but it might be needed but actually probably not
    # Parse connection argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    args = parser.parse_args()
    if args.connect:
        connection_string = args.connect
    
    
    # Connect to the Vehicle
    print "Connecting"
    # referenced elsewhere in the file
    global vehicle
    
    # once heartbeat has been adjusted, timeout needs to be set to 5
    # location of USB connection to pixhawk
    # connection_string       = '/dev/ttyACM0'
    
    connection_string = sitl.connection_string()
    vehicle = connect(connection_string, wait_ready=True) #heartbeat_timeout = 30)
    
    print 'connected to vehicle'
    
    print "Basic pre-arm checks"
    # Don't let the user try to fly while autopilot is booting
    if vehicle.mode.name == "INITIALISING":
        print "Waiting for vehicle to initialise"
        time.sleep(1)
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
    
    
    # need to sort out home location setting
    
# =============================================================================
#     # set home location
#     vehicle.home_location=vehicle.location.global_frame
# =============================================================================
    
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")


    # Sanity Checks
    print " Type: %s" % vehicle._vehicle_type
    print " Armed: %s" % vehicle.armed
    print " System status: %s" % vehicle.system_status.state
    print " GPS: %s" % vehicle.gps_0
    print " Alt: %s" % vehicle.location.global_relative_frame.alt
    print " Mode: %s" % vehicle.mode
    print " Home Location: %s" % str(vehicle.home_location)
    
    
    # completes command
    print'DONE'
    


def getHeading():
    print(repr(vehicle.heading))
    print 'DONE'

def getPosition():
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    print lat, lon
    try:
        print drone_position_notify
    except:
        pass
    
    # performs comparison between waypoint and current position
    try:
        lat_waypoint = drone_position_notify[0]
        lon_waypoint = drone_position_notify[1]
        if (round(lat_waypoint, 5) == round(int(lat), 5) and round(lon_waypoint, 5) == round(int(lon), 5)):
            print 'DONE NOTIFY'
        else:
            print 'not reached!' 
    except:
        print ' falied ' 
        pass
    print 'DONE'

def getAltitude():
    # returns altitude above mean sea level
    alt = vehicle.location.global_frame.alt
    print alt
    print 'DONE'

def getStatus():
    print vehicle.system_status
    print 'DONE'

def getHome():
    print vehicle.home_location
    print 'DONE'

def setWaypoint(position):
    try:
        position[2]
    except:
        position.append(int(vehicle.location.global_frame.alt))
    
    lat, lon, alt = position
    # convert to integers as input is 
    lat = float(lat)
    lon = float(lon) 
    alt = float(alt)
    
    # saves current waypoint co-ordinates
    drone_position_notify = [lat, lon]
    
    
    # converts to co-ord system relative to home point
    point = LocationGlobalRelative(lat,lon,alt)
    vehicle.simple_goto(point)
    print 'moving to', lat, '', lon, '', alt
    print 'DONE'
    
def setHeading(heading_relative):
    # retrieves values froms args list
    heading = int(heading_relative[0])
    relative = False
    # unsure about the behavour of this http://python.dronekit.io/guide/copter/guided_mode.html#guided-mode-copter-set-yaw
    
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        5,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def startTakeoffSequence():
    # arming vehicle and making sure it is armed, if it fails then nothing else will work. 
    for i in range(0,100):
        vehicle.armed = True
        print "Armed: %s" % vehicle.armed
        if vehicle.armed == True:
            break
    
    print 'TAKING OFF'
    vehicle.simple_takeoff(10)
    print 'DONE'
	# Set the controller to take-off and reach a safe altitude (e.g. 20ft)

def startLandingSequence():
    # should probably check the parameters for this..
    vehicle.mode = VehicleMode("RTL")
    print 'Return to Land executed'
    print 'DONE'


def notification(fn):
    # Notification functions
    def location_callback(self, attr_name, value):
        print "Location (Global): ", value

    if fn == 'LOCATION':
        # Add a callback `location_callback` for the `global_frame` attribute.
        vehicle.add_attribute_listener('location.global_frame', location_callback)
    


# end function
    
    

while 1:
    # looks for command and optional arguments 
    cmd = raw_input('>').split()
    if cmd[0] == "connection":
        connection()
    if cmd[0] == "getHeading":
        getHeading()
    if cmd[0] == "getPosition":
        getPosition()
    if cmd[0] == "getPosition":
        getPosition()
    if cmd[0] == "getAltitude":
        getAltitude()
    if cmd[0] == "setWaypoint":
        setWaypoint(cmd[1:])
    if cmd[0] == "setHeading":
        setHeading(cmd[1:])
    if cmd[0] == "startTakeoffSequence":
        startTakeoffSequence()
    if cmd[0] == "startLandingSequence":
        startLandingSequence()
    if cmd[0] == "getStatus":
        getStatus()
    if cmd[0] == "notification":
        notification()

#    if cmd == "telemetryTransmit":
#        telemetryTransmit(args)

    if cmd == "onActionCompleted":
        print 'hi'
    
    if cmd == "exit":
        # sitl stuff
        try:
            sitl.stop()
        except:
            print 'no sitl!'
        sys.exit()
