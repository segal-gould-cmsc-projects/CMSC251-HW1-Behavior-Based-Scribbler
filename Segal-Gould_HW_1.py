# This controller implements behaviors and an fixed priority arbitration
# for them.  Assumes existence of single configured robot.
# Behaviors are implemented as objects.
# Author:  Noah Segal-Gould

from Myro import *
from random import random
from math import *

###############################################################################

class Behavior(object):
    '''High level class for all behaviors.  Any behavior is a
    subclass of Behavior.'''
    NO_ACTION = 0
    def __init__(self):
        self.state = None # Determines current state of the behavior
        # This governs how it will responds to percepts.
    def check(self):
        '''Return True if this behavior wants to execute
        Return False if it does not.'''
        return False
    def run(self):
        '''Execute whatever this behavior does.'''
        return
###############################################################################

###############################################################################


class Avoid(Behavior):
    '''Behavior to avoid obstacles.  Simply turns away.'''
    TURN_LEFT = 1
    TURN_RIGHT = 2
    BACKUP_SPEED = 0.5
    BACKUP_DUR = 0.5
    
    def __init__(self):
        self.state = Avoid.NO_ACTION
        self.obstacleThresh = 2500 ## Higher threshold -> more sensitivity in avoidance.
        self.turnspeed = 0.5
        self.turndur = 0.6

    def check(self):
        '''see if there are any obstacles.  If so turn other direction'''
        L, C, R = getObstacle()
        print("Avoid obst", L, C, R)
        
        pxs, _, _ = getBlob() ## Only grab the blobbed pixels.
        
        ## Added: Avoid if stalled; Avoid if the pylon is not present.
        if getStall() == 1 or (pxs <= 50 and (L > self.obstacleThresh or C > self.obstacleThresh)):
            self.state = Avoid.TURN_RIGHT
            return True
        elif getStall() == 1 or (pxs <= 50 and R > self.obstacleThresh):
            self.state = Avoid.TURN_LEFT
            return True
        else:
            self.state = Avoid.NO_ACTION
        return False


    def run(self):
        '''see if there are any obstacles.  If so turn other direction'''
        print('Avoid')
        backward(self.BACKUP_SPEED, self.BACKUP_DUR) # back up a bit
        if self.state == Avoid.TURN_RIGHT:
            print('turning right')
            turnRight(self.turnspeed, self.turndur)
        elif self.state == Avoid.TURN_LEFT:
            print('turning left')
            turnLeft(self.turnspeed, self.turndur)


###############################################################################



###############################################################################


class Wander(Behavior):
    '''
    Behavior to wander forward.  
    Heads in direction that varies a bit each time it executes.
    '''
    WANDER = 1
    OBSTACLE_THRESH = 1500 ## Higher threshold -> more eagerness in avoidance.
    MAX_SPEED = 1.0 
    MIN_SPEED = 0.1
    DSPEED_MAX = 0.1 # most speed can change on one call
    IMG_WIDTH = 427 ## Width of the Scribbler camera's resolution.
    PYLON_POS = 0 ## Default position for pylon tracker.
    
    def __init__(self):
        self.state = Wander.NO_ACTION
        self.lspeed = self.MAX_SPEED # speed of left motor
        self.rspeed = self.MAX_SPEED # speed of right motor

    def check(self):
        '''see if there are any possible obstacles.  If not, then wander.'''
        L, C, R = getObstacle()
        
        print("Wander obst", L, C, R, (L+C+R)/3.0)
        pxs, _, _ = getBlob()
        
        ## Added: continue wandering even when the pylon is present -> ultimately the only thing it does not avoid.
        if (L+C+R)/3.0 < self.OBSTACLE_THRESH or pxs > 150:
            self.state = self.WANDER
            return True
        else:
            self.state = self.NO_ACTION
            return False

    def run(self):
        '''Modify current motor commands by a value in range [-0.25,0.25].'''
        
        ## Helper function to keep track of where the pylon last was.
        def setPylonPos():
            ## show(takePicture("blob"))
            pxs, avg_x, _ = getBlob()
            if pxs > 150: ## A threshold to take care of false positives.
                
                ## Pylon centered on the left.
                if avg_x < self.IMG_WIDTH//2:
                    self.PYLON_POS -= 1
                    beep(0.5, 1479.98) ## Play an F# (like the Mac).
                    print("Estimated pylon position: " + str(self.PYLON_POS))
                
                ## Pylon centered on the right.
                if avg_x > self.IMG_WIDTH//2:
                    self.PYLON_POS += 1
                    beep(0.5, 1479.98) ## Play an F# (like the Mac).
                    print("Estimated pylon position: " + str(self.PYLON_POS))
        
        print('Wander')
        dl = (2 * random() - 1) * self.DSPEED_MAX
        dr = (2 * random() - 1) * self.DSPEED_MAX
        self.lspeed = max(self.MIN_SPEED, min(self.MAX_SPEED, self.lspeed+dl))
        self.rspeed = max(self.MIN_SPEED, min(self.MAX_SPEED, self.rspeed+dr))
        
        ## Check where the pylon was before moving.
        prevPylonPos = self.PYLON_POS
        
        ## Update the estimated position of the pylon.
        setPylonPos()
        
        ## Turn in the direction of the pylon or proceed normally if within +- 2.
        if self.PYLON_POS < prevPylonPos - 2:
            motors(-abs(self.lspeed), abs(self.rspeed))
            print("turning left")
        elif self.PYLON_POS > prevPylonPos + 2:
            motors(abs(self.lspeed), -abs(self.rspeed))
            print("turning right")
        else:
            motors(self.lspeed, self.rspeed)
        
        ## Update the estimated position of the pylon after moving.
        setPylonPos()
        
        


class Controller(object):
    
    def __init__(self, configureBlobbing=True, address="/dev/tty.Fluke2-094E-Fluke2"):
        '''Create controller for object-finding robot.'''
        
        init(address)
        
        if configureBlobbing:
            configureBlob(0, 254, 112, 117, 152, 165)
        else:
            pass

        self.message = "?"
        self.avoidBehavior = Avoid()
        self.wanderBehavior = Wander()
        # Order implements priority arbitration.
        self.behaviors = [self.avoidBehavior, self.wanderBehavior]
        
    # Decide which behavior, in order of priority
    # has a recommendation for the robot
    def arbitrate(self):
        for behavior in self.behaviors:
            wantToRun = behavior.check()
            if wantToRun:
                behavior.run()
                return # no other behavior runs

    def run(self):
        setForwardness('fluke-forward')
        # This is the simplest loop.  You may like to stop the bot in other ways.
        for seconds in timer(180): # run for 3 minutes
            self.arbitrate()
        stop()

if __name__ == "__main__":
    ctl = Controller()
    ctl.run()
