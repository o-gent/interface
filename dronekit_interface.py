#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 14 09:48:06 2018
Dronekit interface
Passes and recieves strings from dronekit_functions which interacts with dronekit
@author: OliG
"""

import subprocess
import time

class FCInterface:

    def __init__(self):
        
        self.timeoutLines = 1000
        self.notificationCallbacks = {} 	# dictionary, populated by setNotificationCallback
        self.notificationQueue = [] 		# last at end
        
        # opens cli running python2 dronekit functions
        self.py2 = subprocess.Popen(['python','-u','dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)
        print ("FCInterface initialised")

    def interface(self, command):
        """
        pass a function name to dronekit_functions
        returns single line string resulting from function. When DONE is passed function is considered complete and 
        script moves on.
        """
        print("")
        print('Sending cmd:', command)
        self.py2.stdin.write(command + '\n')
        self.py2.stdin.flush()
        print('Listening...')
        
        returnLine = None 	# always the last line before the current one
        numLinesRead = 0
        stackHeight = 0 	# number of execution levels = number of COMMANDs printed - number of DONEs printed
        
        while numLinesRead < self.timeoutLines:
			# self.waypoint_reached = False     (no longer required)
			
			# check integrity of execution stack
            #if stackHeight == 0:
                #print("Erorr: stack height is non-zero = ", stackHeight)
            
            # read subprocess output
            read = self.py2.stdout.readline()[:-1] # removes final newline character
            # print line through to interface
            print(str(numLinesRead) + '| ' + read)
            
            # handles
            if read.startswith('NOTIFY'):
                self.notificationQueue.append(read[7:])
            elif read.startswith('COMMAND'):
                stackHeight += 1
                
            elif read.startswith('DONE'):
                stackHeight -= 1
                return returnLine
                break
            
            returnLine = read
            numLinesRead += 1
        
        # only reached if DONE is not returned
        print("Command timed out after", numLinesRead, "lines read")
        
    def setNotificationCallback(self, name, fn):
        self.notificationCallbacks[name] = fn

    def connection(self):
        """
        connection to vehicle through interface and performs basic setup including setting home.
        won't complete until connection has complete
        """
        self.interface('connection')
        return True

    def getHeading(self):
        """
        no args, returns heading in degress from North
        """
        ans = self.interface('getHeading')
        return ans 
    
    def getPosition(self):
        """
        returns lat and lon as two vars
        """
        ans = self.interface('getPosition')
        # converts single string to two ints
        try:
            position = ans.split()
            lat = float(position[0])
            lon = float(position[1])
            return lat, lon
        except:
            print('getPosition failed')
            return 0, 0
        
    def getAltitude(self):
        """
        returns altitude above mean sea level in meters
        """
        ans = self.interface('getAltitude')
        return ans

    def setWaypoint(self, lat, lon, *args):
        """
        IN: target LAT, LON and optional altitude, if no altittude passed it will use current altitude
        """
        try:
            alt = args[0]
        except:
            alt = self.getAltitude()
        self.interface('setWaypoint' + ' ' + str(lat) + ' ' + str(lon) + ' ' + str(alt))

    def setHeading(self, heading):
        """
        travels in set heading (in degrees from North.)
        """
        self.interface('setHeading' + ' ' + str(heading))

    def startTakeoffSequence(self):
        """
        arms the copter then takes off to 10 meters
        """
        self.interface('startTakeoffSequence')

    def startLandingSequence(self):
        """
        starts landing copter
        """
        self.interface('startLandingSequence')
        
    def onActionCompleted(self, fn):
        self.waypoint_reached_fn = fn
        # Non-blocking callback function
        # Called once by command.py during aircraft boot
        # Sets up a notification so that, every time a commanded action is completed (e.g. waypoint/heading reached, take-off completed), function fn will be called-back



# -------------------------------------------
# example mission

numWPsDone = 0
def waypointReachedCallback():
	global numWPsDone, lat, lon

	print("Waypoint reached (callback)")
	numWPsDone += 1
	print("WPs completed:", numWPsDone)

	if numWPsDone >= 5:
		print(numWPsDone, "WPs complete, starting landing...")
		fci.startLandingSequence()
	else:
		print("Setting next WP...")
		fci.setWaypoint(lat, lon + 0.0002, 300)

# init interface
fci = FCInterface()
fci.setNotificationCallback('waypointReached', waypointReachedCallback) # set callback reference
time.sleep(4)

# [TODO] do simulator here instead

# connect and take off
fci.connection()
fci.startTakeoffSequence()
time.sleep(10)

# get current position
lat, lon = fci.getPosition()
print('Initial position:', lat, lon)

# set initial waypoint
fci.setWaypoint(lat + 0.0001, lon, 600)

# keep checking position and handling notifications
for i in range(10000):
	# handle notifications
	while len(fci.notificationQueue):
		note = fci.notificationQueue.pop(0)
		print("Handling notification", note)

		if note in fci.notificationCallbacks:
			fci.notificationCallbacks[note]()

	# update position
	lat, lon = fci.getPosition()
	print('Pos checked cyclically:', lat, lon)

	# wait
	time.sleep(1)
