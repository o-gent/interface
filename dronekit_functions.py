# -*- coding: utf-8 -*-
"""
Created on Fri May 18 16:22:45 2018
PYTHON 2
list of functions interacting with dronekit, accepting strings as input to execute functions
@author: OliG
"""
from dronekit import connect, VehicleMode, LocationGlobal
import time
drone_position_notify = [0,0]

def connection():
    
    # sitl stuff   ###########################
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    print 'sitl started'
    ##########################################
    
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
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print " Waiting for home location ..."
            time.sleep(0.2)
    print "\n Home location: %s" % vehicle.home_location
    
    
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
    
    # set parameters
    vehicle.parameters['WP_YAW_BEHAVIOR']=0
    
    # completes command
    print'DONE'
    
def getHeading():
    print(repr(vehicle.heading))
    print 'DONE'

def getPosition():
    lat = float(vehicle.location.global_frame.lat)
    lon = float(vehicle.location.global_frame.lon)
    # fails if a waypoint hasn't been set yet 
    try:
        print 'target waypoint: ', drone_position_notify
    except:
        pass
    # performs comparison between waypoint and current position
    try:
        lat_waypoint = float(drone_position_notify[0])
        lon_waypoint = float(drone_position_notify[1])
        lat_check = (lat_waypoint - lat)/lat_waypoint
        lon_check = (lon_waypoint - lon)/lon_waypoint
        lat_percent = 5e-08
        lon_percent = 5e-08
        
        if (lat_check < lat_percent and lon_check < lon_percent):
            print 'NOTIFY'
        else:
            print 'not reached!' 
    except:
        print 'failed' 
    print lat, lon
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
    # check if altitude has been passed
    try:
        position[2]
    except:
        alt = float(vehicle.location.global_frame.alt)
    lat, lon, alt = position
    print position
    # convert to integers as input is 
    lat = float(lat)
    lon = float(lon) 
    alt = float(alt)
    # saves current waypoint co-ordinates
    global drone_position_notify
    drone_position_notify = [lat, lon]
    # converts to co-ord system relative to home point
    point = LocationGlobal(lat,lon,alt)
    vehicle.simple_goto(point)
    print 'moving to', lat, '', lon, '', alt
    print 'DONE'
    
def setHeading(heading_relative):
    # retrieves values froms args list
    heading = int(heading_relative[0])
    is_relative=0
    yaw_speed = 20
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # ignore
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        yaw_speed,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
def startTakeoffSequence():
    # arming vehicle and making sure it is armed, if it fails then nothing else will work. 
    for i in range(0,500):
        vehicle.armed = True
        time.sleep(0.5)
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
    try:
        cmd = raw_input('>').split()
        if cmd[0] == "connection":
            connection()
        if cmd[0] == "getHeading":
            getHeading()
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
        if cmd[0] == "getHome":
            getHome()
        
        if cmd == "exit":
            # sitl stuff
            try:
                sitl.stop()
            except:
                print 'no sitl!'
    except:
        time.sleep(0.1)
