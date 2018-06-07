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

# =============================================================================
#     def __init__(self):
#         
#         # opens cli running python2 dronekit functions
#         self.py2 = subprocess.Popen(['python','-u','dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)
#         #self.py2 = subprocess.Popen([pycmd, '-u','dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)
#         print ("FCInterface initialised")
# 
#     def interface(self, command):
#         """
#         pass a function name to dronekit_functions
#         returns single line string resulting from function. When DONE is passed function is considered complete and 
#         script moves on.
#         """
#         print('writing------------------------------------------------------------------------')
#         self.py2.stdin.write(command + '\n')
#         self.py2.stdin.flush()
#         print('reading------------------------------------------------------------------------')
#         for i in range(0,100):
#             
#             #self.waypoint_reached = False
#             
#             
#             read = self.py2.stdout.readline()
#             
#             if read.startswith('NOTIFY'):
#                 try:
#                     self.waypoint_reached_fn()
#                 except:
#                     pass
#             
#             print(read)
#             # keeps last line printed
#             cmdreturn = read
#             
#             if read.startswith('DONE'):
#                 return(cmdreturn)
#                 break
# =============================================================================
    def __init__(self):
        self.timeoutLines = 1000
        self.notificationCallbacks = {} 	# dictionary, populated by setNotificationCallback
        self.notificationQueue = [] 		# last at end
        
        # opens cli running python2 dronekit functions
        self.py2 = subprocess.Popen(['python','-u','dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)
        #self.py2 = subprocess.Popen([pycmd, '-u','dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)
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
        no args, returns lat and lon as two vars
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
        returns altitude above mean sea level in meters, no args
        """
        ans = self.interface('getAltitude')
        return ans

    def setWaypoint(self, lat, lon, alt):
        """
        uses simple_goto
        """
        self.interface('setWaypoint' + ' ' + str(lat) + ' ' + str(lon) + ' ' + str(alt))

    def setHeading(self, heading, *args):
        """
        Pass a heading in degrees from North. I think if you pass True as a second argument the copter will return to 
        using a relative heading (it may travel faster)
        """
        try: 
            relative = str(args[0])
        except:
            relative = ''
        self.interface('setHeading' + ' ' + str(heading) + ' ' + relative)

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

FCI = FCInterface()
time.sleep(4)

FCI.connection()
FCI.startTakeoffSequence()
time.sleep(30)

FCI.getAltitude()

FCI.getPosition()
lat, lon = FCI.getPosition()

print(lat, lon)

time.sleep(5)

FCI.setWaypoint(float(lat - 0.001), lon, 600)
for i in range(10000):
    lat, lon = FCI.getPosition()
    time.sleep(2)
    
FCI.startLandingSequence()
