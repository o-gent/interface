# -*- coding: utf-8 -*-
"""
Created on Fri May 18 16:22:45 2018
PYTHON 2
list of functions interacting with dronekit, accepting strings as input to execute functions
@author: OliG
"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time, argparse

# essentially WP position
drone_position_notify = [0,0]

def connection():
	# sitl stuff   ###########################
	import dronekit_sitl
	sitl = dronekit_sitl.start_default()
	print('sitl started')
	##########################################
	
	# (seems pointless?)
	# literally no clue what these 5 lines do but it might be needed but actually probably not
	# Parse connection argument
	# parser = argparse.ArgumentParser()
	# parser.add_argument("-c", "--connect", help="connection string")
	# args = parser.parse_args()
	# if args.connect:
	#     connection_string = args.connect

	# print("Args:", args)
	# print("Connection string:", connection_string)
	
	
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
	lat = float(vehicle.location.global_frame.lat)
	lon = float(vehicle.location.global_frame.lon)
	
	print drone_position_notify

	# check if waypoint reached
	lat_waypoint = float(drone_position_notify[0])
	lon_waypoint = float(drone_position_notify[1])
	lat_check = abs((lat_waypoint - lat)/lat) 	# use target position here - much less likely to = 0 
	lon_check = abs((lon_waypoint - lon)/lon)

	print 'percentage to WP: ' + str(lat_check * 100) + '% ' + str(lon_check * 100) + '%'
	
	lat_percent = 5e-08
	lon_percent = 5e-08
	
	if lat_check < lat_percent and lon_check < lon_percent:
		print 'NOTIFY waypointReached'
	else:
		print 'WP not yet reached!'

	# return position
	print str(lat) + ' ' + str(lon)
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
	# if no position given
	try:
		position[2]
	except:
		position.append(float(vehicle.location.global_frame.alt))
	
	lat, lon, alt = position
	lat = float(lat)
	lon = float(lon) 
	alt = float(alt)
	
	# saves current waypoint co-ordinates
	global drone_position_notify
	drone_position_notify = [lat, lon]
	
	# converts to co-ord system (LocationGlobalRelative relative to home point?)
	point = LocationGlobal(lat,lon,alt)
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

	print 'DONE'

def startTakeoffSequence():
	# arming vehicle and making sure it is armed, if it fails then nothing else will work. 
	for i in range(0,100):
		vehicle.armed = True
		print "Armed: %s" % vehicle.armed

		# wait for critical motor arming (arm seems to take a while and doesn't block)
		# if these dont complete the aircraft will not move in SITL
		time.sleep(0.5)

		if vehicle.armed == True:
			break
	
	print 'taking off...'
	vehicle.simple_takeoff(10)

	print 'DONE'
	# Set the controller to take-off and reach a safe altitude (e.g. 20ft)



def startLandingSequence():
	# should probably check the parameters for this..
	vehicle.mode = VehicleMode("RTL")
	print 'Return to Land executed'
	print 'DONE'

# def notification(fn):
# 	# Notification functions
# 	def location_callback(self, attr_name, value):
# 		print "Location (Global): ", value

# 	if fn == 'LOCATION':
# 		# Add a callback `location_callback` for the `global_frame` attribute.
# 		vehicle.add_attribute_listener('location.global_frame', location_callback)

# end function
	
while 1:
	# FCI-F = flight control interface, functions.py
	args = raw_input('FCI-F> ').split()
	cmd = args.pop(0) # remove first element (command)

	print("COMMAND " + cmd)

	# commands are exclusive so elif will be much faster than if

	if cmd == "connection":
		connection()
	elif cmd == "getHeading":
		getHeading()
	elif cmd == "getPosition":
		getPosition()
	elif cmd == "getPosition":
		getPosition()
	elif cmd == "getAltitude":
		getAltitude()
	elif cmd == "setWaypoint":
		setWaypoint(args)
	elif cmd == "setHeading":
		setHeading(args)
	elif cmd == "startTakeoffSequence":
		startTakeoffSequence()
	elif cmd == "startLandingSequence":
		startLandingSequence()
	elif cmd == "getStatus":
		getStatus()
	elif cmd == "notification":
		notification()
	elif cmd == "getHome":
		getHome()

	# elif cmd == "telemetryTransmit":
	#     telemetryTransmit(args)

	elif cmd == "onActionCompleted":
		print 'hi'
	
	elif cmd == "exit":
		# sitl stuff
		try:
			sitl.stop()
		except:
			print 'no sitl!'

		# exit by simply ending the script
		break
 
 		# exit manually
		sys.exit()

	else:
		print('Unkown command:', cmd)

	# keep script alive in between commands
	time.sleep(0.1)