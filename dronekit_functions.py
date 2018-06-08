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
import sys


approxDegsPerMetre = 9e-06
m2d = approxDegsPerMetre	# multiply metres by this to get degrees (D = ~9e-06 x M)
d2m = 1/approxDegsPerMetre  # multiply degrees by this to get metres (M = D / ~9e-06)

# essentially WP position
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
	vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
	
	# completes command
	print'DONE'

def getHeading():
	print(repr(vehicle.heading))
	print 'DONE'

def getPosition():
	lat = float(vehicle.location.global_frame.lat)
	lon = float(vehicle.location.global_frame.lon)
	
	#print 'WP: ' + str(drone_position_notify)

	# check if waypoint reached
	max_d_m = 0.5 			# threshold in m
	max_d = max_d_m * m2d 	# threshold in deg, see calcs.py
	lat_wp = float(drone_position_notify[0])
	lon_wp = float(drone_position_notify[1])
	lat_d = abs(lat_wp - lat)
	lon_d = abs(lon_wp - lon)
	#print('OFFSET: ' + str([lat_d * d2m, lon_d * d2m]) + 'm')

	#pythag_d = (lat_d**2 + lon_d**2)**0.5
	#print("Aprrox.", pythag_d, "deg = ", pythag_d * d2m, "m to WP (max_d=", max_d, ")")

	if lat_d < max_d and lon_d < max_d:
		print 'NOTIFY waypointReached'
	#else:
		#print 'WP not yet reached!'

	# return position
	print str(lat) + ' ' + str(lon)
	print 'DONE'

def getAltitude():
	# returns altitude above home location (location.alt is above mean sea level)
	alt = (vehicle.location.global_frame.alt) - float(vehicle.home_location.alt)
	print alt
	print 'DONE'

def getStatus():
	print vehicle.system_status
	print 'DONE'

def getHome():
	print vehicle.home_location
	print 'DONE'

def setWaypoint(position):
	global drone_position_notify

	# if no position given
	try:
		position[2]
	except:
		position.append(float(vehicle.location.global_frame.alt))
	
	# altitude given is relative to ground/home
	groundHeight = float(vehicle.home_location.alt)

	lat, lon, alt = position
	lat = float(lat)
	lon = float(lon) 
	alt = float(alt) + groundHeight	
	
	# saves current waypoint co-ordinates
	drone_position_notify = [lat, lon]

	# print distance to target
	# lat_c = float(vehicle.location.global_frame.lat) # current location
	# lon_c = float(vehicle.location.global_frame.lon)
	# lat_d = abs(lat - lat_c)
	# lon_d = abs(lon - lon_c)
	# print('OFFSET: ' + str([lat_d * d2m, lon_d * d2m]) + 'm')
	
	# converts to co-ord system (LocationGlobalRelative relative to home point?)
	point = LocationGlobal(lat,lon,alt)
	vehicle.simple_goto(point)
	print 'waypoint set:', lat, '', lon, '', alt
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
	
	print 'DONE'

def startTakeoffSequence():
	# arming vehicle and making sure it is armed, if it fails then nothing else will work. 
	for i in range(0,500):
		vehicle.armed = True

		# wait for critical motor arming (arm seems to take a while and doesn't block)
		# if these dont complete the aircraft will not move in SITL
		time.sleep(0.5)

		print "Armed: %s" % vehicle.armed

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
	try:
		# FCI-F = flight control interface, functions.py
		args = raw_input('FCI-F> ').split()
		cmd = args.pop(0) # remove first element (command)
	except:
		sys.exit()

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