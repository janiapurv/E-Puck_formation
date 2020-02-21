
# coding: utf-8

# In[1]:



from epuck import epuck
import time
import math
import random

class EPuckHandler:
    def __init__(self, name,com, start_pos, orientation, ANG_TOL = .07, LIN_TOL = .01, speed = 2, rotSpeed = .5):
        
        self.name = name
        
        self.targReached = True
        self.orientation = orientation
        self.epk = epuck(orientation,port='COM'+str(com), startPos=start_pos)
        self.ANG_TOL = .03
        self.LIN_TOL = .01
        self.speed = speed;
        self.rotSpeed = rotSpeed;
        self.target = start_pos;
        self.position = start_pos
        self.d = self.calcDir(self.target,self.position)
        self.count = 0;
    
    def calcDir(self, targ, od):
        dx = (targ[0] - od[0]);
        dy = (targ[1] - od[1]);
        targTheta = 0
        if dx==0:
            if dy < 0:
                targTheta = 3/2*math.pi
            elif dy > 0:
                targTheta = 1/2*math.pi
        elif dy == 0:
            if dx < 0:
                targTheta = math.pi
            elif dx > 0:
                targTheta = 0;
        elif dx > 0 and dy > 0:
            targTheta = math.atan(dy/dx);
        elif dx < 0 and dy > 0:
            targTheta = math.pi - math.atan(dy/(-dx))
        elif dx < 0 and dy < 0:
            targTheta = math.pi + math.atan((-dy)/(-dx))
        elif dx > 0 and dy < 0:
            targTheta = 2*math.pi - math.atan((-dy)/(dx))

        dist = math.sqrt(math.pow(od[0] - targ[0],2) + math.pow(od[1] - targ[1],2));  

        return (targTheta,dist)
    
    def stop(self):
        self.epk.setRobotVelocity([0,0])

    def angDist(self,theta1,theta2):
        return min(abs(theta1-theta2), 2*math.pi - abs(theta1-theta2))

    def TurnToTarget(self,theta, targTheta):
        
        if self.angDist(theta,targTheta) > self.ANG_TOL:
            if abs(theta - targTheta) > math.pi:
                if theta < targTheta:
                    self.epk.setRobotVelocity([0,self.rotSpeed])
                elif theta > targTheta:
                    self.epk.setRobotVelocity([0,-self.rotSpeed])
            else :
                if theta < targTheta:
                    self.epk.setRobotVelocity([0,-self.rotSpeed])
                elif theta > targTheta:
                    self.epk.setRobotVelocity([0,self.rotSpeed])


    # assumes is already facing target
    def MoveToTarget(self):

        self.epk.setRobotVelocity([self.speed,0]);


    def avoid():
        sens = epk.getProximitySensor();
        print(sens)
        stall = 0;
        while sens[0]>stopDist or sens[1]>stopDist or sens[6]>stopDist or sens[7]>stopDist:
            epk.setRobotVelocity([0,0])
            sens = sens = epk.getProximitySensor();

    def ReachedTarget(self):
        self.stop()
        self.targReached = True


    def setNewTarget(self,target):
        self.target = target
        self.targReached = False

    def Update(self,i):
        print("*********")
        print("update: " +str(i))
        self.count = self.count + 1
        self.position = self.epk.getOdometry()
        self.d = self.calcDir(self.target,self.position)
        print("Distance Left for " +str(i)+": " + str(((self.position[0] - self.target[0])**2 + (self.position[1] - self.target[1])**2)**0.5))
        print(self.position)
        print("Current target: "+str(self.target))
        if self.d[1] > self.LIN_TOL:
            
            if self.position[2] > self.ANG_TOL + self.d[0] or self.position[2] < self.d[0] - self.ANG_TOL:
                print("turning towards")
                self.TurnToTarget(self.position[2],self.d[0])
            else:
                print("move to target")
                self.MoveToTarget()

        else:
            self.ReachedTarget()


    def getTravelled(self):
        return self.epk.getTravelled()

    def isFinished(self):
        return self.targReached

    def getName(self):
        return self.name