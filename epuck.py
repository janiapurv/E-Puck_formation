#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Communication is implemented based on ASCII mode: http://www.gctronic.com/doc/index.php/Advanced_sercom_protocol
#
# Example:
# from epuck import epuck
#
# epk = epuck()
#
# while True:
#     cmd = input("Enter command or 'exit':")
#         # for Python 3
#     if cmd == 'exit':
#         self.close()
#     else:
#         out = epk.getAccelerometer()
#         print('Receiving: '+str(out))
#         out = epk.getMotorSpeed()
#         print('Receiving: '+str(out))
#
# Author: Payam Ghassemi <pnpayam AT gmail DOT com>
# Copyright 2018-2019 Payam Ghassemi
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.

import serial
import math


## Camera parameters
IMAGE_FORMAT = 'RGB_365'
CAMERA_ZOOM = 8

## Epuck dimensions
# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3

# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.053
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
ALPHA = 0.180 # deg/step

BETA = 360/ALPHA #steps per revolution
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/BETA)    #/1000) 0.000125 meters per step (m/steps)

END = '\x0d\x0a'

class epuck:
    def __init__(self,orientation, port='COM12',startPos=(0,0,0),baudrate=115200,debug=False):
        self.MOT_STEP_DIST = MOT_STEP_DIST
        ALPHA = 0.180 # deg/step
        BETA = 360/ALPHA #steps per revolution
        self.MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/BETA)    #/1000) 0.000125 meters per step (m/steps)
        if(orientation):
            print("orientation 1")
            ALPHA = .38
            BETA = 360/ALPHA #steps per revolution
            self.MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/BETA)    #/1000) 0.000125 meters per step (m/steps)
        self.port = port
        self.baudrate = baudrate
        self.debug = debug
        self.orientation = orientation
        # open the serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        if ser.isOpen():
            print(ser.name + ' is open...')
        self.ser = ser
        self.leftStepsPrev = 0.
        self.rightStepsPrev = 0.

        self.x_pos = startPos[0]  # Expressed in meters.
        self.y_pos = startPos[1]  # Expressed in meters.
        self.theta = startPos[2]   # Expressed in radiant.
        self.travelled = 0
        self.resetRobot()

    def close(self):
        """
        Closes the serial communication.
        """
        self.ser.close()
        exit()

    def resetRobot(self):
        """
        Resets the e-puck.
        """
        cmd = 'R'
        self.ser.write(cmd.encode()+END.encode())
        return 0

    def getData(self,cmd):
        """
        Gets infromation from the e-puck based on the given "cmd".
        Input Param:
            cmd: command based on Advanced Sercom Protocol.
        """
        self.ser.write(cmd.encode()+END.encode())
        out = self.ser.readline()

        if(out == ""):
            raise IOError("communication failed")
        return out

    def sendCmd(self,cmd):
        """
        Sends the command "cmd" to the e-puck via serial port.
        Input Param:
            cmd: command based on Advanced Sercom Protocol.
        """
        self.ser.write(cmd.encode()+END.encode())
        out = self.ser.readline()
        return out

    def getOdometry(self):
        """
        Returns odometry information. The ROS package is used as a refenerece: https://github.com/gctronic/epuck_driver/blob/master/scripts/epuck_driver.py
        """
        last = [self.x_pos,self.y_pos]
        mp = self.getMotorSteps()
         # Get a list since tuple elements returned by the function are immutable.
        if(mp is None):
            print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
            print("missed odometry: stopping bot")
            self.stop()
            return [self.x_pos,self.y_pos,self.theta]
        motor_pos = list(mp)
        if(self.orientation):
            motor_pos[1] = -motor_pos[1]
        print(motor_pos)
        
        self.leftStepsDiff = motor_pos[0]*self.MOT_STEP_DIST - self.leftStepsPrev    # Expressed in meters.
        self.rightStepsDiff = motor_pos[1]*self.MOT_STEP_DIST - self.rightStepsPrev  # Expressed in meters.
        #print "left, right steps diff: " + str(self.leftStepsDiff) + ", " + str(self.rightStepsDiff)

        self.deltaTheta = (self.rightStepsDiff - self.leftStepsDiff)/(WHEEL_DISTANCE) # Expressed in radiant.
        self.deltaSteps = (self.rightStepsDiff + self.leftStepsDiff)/2  # Expressed in meters.
        #print "delta theta, steps: " + str(self.deltaTheta) + ", " + str(self.deltaSteps)

        self.x_pos += self.deltaSteps*math.cos(self.theta + self.deltaTheta/2)  # Expressed in meters.
        self.y_pos += self.deltaSteps*math.sin(self.theta + self.deltaTheta/2)  # Expressed in meters.
        self.theta += self.deltaTheta   # Expressed in radiant.
        #print "x, y, theta: " + str(self.x_pos) + ", " + str(self.y_pos) + ", " + str(self.theta)
        if self.theta > 2*math.pi:
            self.theta = self.theta - 2*math.pi
        elif self.theta < 0:
            self.theta = self.theta + 2*math.pi
        self.leftStepsPrev = motor_pos[0]*self.MOT_STEP_DIST  # Expressed in meters.
        self.rightStepsPrev = motor_pos[1]*self.MOT_STEP_DIST    # Expressed in meters.
        self.travelled += ((self.x_pos - last[0])**2 + (self.y_pos - last[1])**2)**.5
        

        return [self.x_pos,self.y_pos,self.theta]

    def getTravelled(self):
        return self.travelled

    def getAccelerometer(self):
        """
        Returns the acceleration sensor data.
        """
        cmd = 'A'
        acc = [-1,-1,-1]
        out = self.getData(cmd)
        out = str(out, 'utf-8')
        if self.debug:
            print(out)
        isStart = False
        if out[0] == 'a':
            j = 0
            for i in range(len(out)):
                if isStart:
                    if out[i] == ',':
                        acc[j] = int(data)
                        j = j + 1
                        isStart = False
                    else:
                        data=data+out[i]
                if out[i] == ',':
                    isStart = True
                    data = ''
            acc[j] = int(data)
        return acc

    def getMotorSteps(self):
        """
        Returns the motor pose.
        """
        cmd = 'Q'
        pos = [-1,-1]
        #print('Steps: ');
        #print(out);
        out = self.getData(cmd)
        out = str(out, 'utf-8')
        if(len(out) == 0):
            return
        if out[0] == 'q':
            isStart = False
            j = 0
            for i in range(len(out)):
                if isStart:
                    if out[i] == ',':
                        pos[j] = int(data)
                        j = j + 1
                        isStart = False
                    else:
                        data=data+out[i]
                if out[i] == ',':
                    isStart = True
                    data = ''
            pos[j] = int(data)
        pos[1] = -pos[1]
        return pos

    def getMotorSpeed(self):
        """
        Returns the speed of the right and left motors.
        """
        cmd = 'E'
        vel = [-1,-1]
        out = self.getData(cmd)
        out = str(out, 'utf-8')
        if self.debug:
            print(out)
        if out[0] == 'e':
            isStart = False
            j = 0
            for i in range(len(out)):
                if isStart:
                    if out[i] == ',':
                        vel[j] = int(data)
                        j = j + 1
                        isStart = False
                    else:
                        data=data+out[i]
                if out[i] == ',':
                    isStart = True
                    data = ''
            vel[j] = int(data)
        return vel

    def stop(self):
        self.setRobotVelocity([0,0])

    def setMotorSpeed(self,velRight=500,velLeft=500):
        """
        Sets the velocity of right and left motors.
        Input Params:
            VelRight: the velocity of the right motor.
            VelRight: the velocity of the left motor.
        """
        cmd = 'D,'+str(velRight)+','+str(velLeft)
        self.sendCmd(cmd)

    def setRobotVelocity(self,vel):
        """
        Controls the velocity of each wheel based on linear and angular velocities.
        Input Params:
            Vel: [linear_vel, angular_vel]
        """
        linear = vel[0]
        angular = vel[1]
        if not (self.orientation):
            # Kinematic model for differential robot.
            print("or 0")

            wl = (linear - (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER
            wr = (linear + (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER

            # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
            velLeft = int(wl * BETA/(2*math.pi))
            velRight = int(wr * BETA/(2*math.pi))
            print("left: " + str(velLeft))
            print("right: " + str(velRight))
            self.setMotorSpeed(velRight,velLeft)
        else:
            print("or 1")
            # Kinematic model for differential robot.
            wl = (linear - (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER
            wr = (linear + (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER

            # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
            velLeft = int(wl * BETA/(2*math.pi))
            velRight = int(wr * BETA/(2*math.pi))
            print("left: " + str(velLeft))
            print("right: " + str(velRight))
            self.setMotorSpeed(velRight, velLeft)


    def getProximitySensor(self):
        """
        Returns an 1x8 array, which contains the values of the eight proximity sensors.
        """
        cmd = 'N'
        proximitySensor = [-1,-1,-1,-1,-1,-1,-1,-1]
        out = self.getData(cmd)
        out = str(out, 'utf-8')
        if self.debug:
            print(out)
        if out[0] == 'n':
            isStart = False
            j = 0
            for i in range(len(out)):
                if isStart:
                    if out[i] == ',':
                        proximitySensor[j] = int(data)
                        j = j + 1
                        isStart = False
                    else:
                        data=data+out[i]
                if out[i] == ',':
                    isStart = True
                    data = ''
            proximitySensor[j] = int(data)
        return proximitySensor

    def getAmbientLightSensor(self):
        """
        Returns the value of the ambient light sensor.
        """
        cmd = 'O'
        ambientLightSensor = [-1,-1,-1,-1,-1,-1,-1,-1]
        out = self.getData(cmd)
        out = str(out, 'utf-8')
        if self.debug:
            print(out)
        if out[0] == 'n':
            isStart = False
            j = 0
            for i in range(len(out)):
                if isStart:
                    if out[i] == ',':
                        ambientLightSensor[j] = int(data)
                        j = j + 1
                        isStart = False
                    else:
                        data=data+out[i]
                if out[i] == ',':
                    isStart = True
                    data = ''
            ambientLightSensor[j] = int(data)
        return ambientLightSensor
