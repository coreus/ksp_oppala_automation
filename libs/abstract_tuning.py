import logging
import abc
import numpy as np

class AbstractTuning(metaclass=abc.ABCMeta):
    def __init__(self, getPosition):
        logging.basicConfig(level=logging.INFO)
        self.threshold = 1
        self.targetPosition = 0
        self.getPosition = getPosition
        self.reset()

    def setTargetPosition(self,target):
        self.targetPosition = target
        self.reset()

    def setThreshold(self,threshold):
        self.threshold = threshold
    
    def reset(self) -> None:
        self.positionT1 = self.getPosition()
        self.deltaPositionT1 = 0
        self.deltaDeltaPositionT1 = 0
        self.deltaDeltaDeltaPositionT1 = 0
    
    #return position when velocity reach 0 or false if that's not going to happen
    def predictPositionAtZeroVelocity(currentPosition, currentVelocity, currentAcceleration, currentJerk) -> float | bool:
        logging.debug('position: ' + str(currentPosition) + ' velocity: ' + str(currentVelocity) + ' acceleration: ' + str(currentAcceleration) + ' jerk: ' + str(currentJerk))

        #solve t when speed = 0
        speedTs = np.sort(np.roots([currentJerk / 2, currentAcceleration, currentVelocity]).real)
        logging.debug(speedTs)
        #only future values of t
        speedTs = speedTs[speedTs>0]
        if len(speedTs) > 0 :
            t = speedTs[0]
            return currentPosition + currentVelocity * t + currentAcceleration * t * t / 2 + currentJerk * t * t * t / 6
        
        return False

    @abc.abstractmethod
    def update(self, k):
        return

    def tune(self) -> None:
        self.positionT0 = self.positionT1
        self.deltaPositionT0 = self.deltaPositionT1
        self.deltaDeltaPositionT0 = self.deltaDeltaPositionT1
        self.deltaDeltaDeltaPositionT0 = self.deltaDeltaDeltaPositionT1

        #position
        self.positionT1 = self.getPosition()
        #velocity
        self.deltaPositionT1 = self.positionT1 - self.positionT0
        #acceleration
        self.deltaDeltaPositionT1 = self.deltaPositionT1 - self.deltaPositionT0
        #jerk
        self.deltaDeltaDeltaPositionT1 = self.deltaDeltaPositionT1 - self.deltaDeltaPositionT0

        self.positionAtZeroVelocity = AbstractTuning.predictPositionAtZeroVelocity(self.positionT1, self.deltaPositionT1, self.deltaDeltaPositionT1,self.deltaDeltaDeltaPositionT1)
    
        if self.positionAtZeroVelocity == False:
            k = 0
            if (self.positionT1 > self.targetPosition and self.deltaPositionT1 > 0) or (self.positionT1 < self.targetPosition and self.deltaPositionT1 > 0):
                k = -1
            if (self.positionT1 > self.targetPosition and self.deltaPositionT1 < 0) or (self.positionT1 < self.targetPosition and self.deltaPositionT1 < 0):
                k = 1
            logging.debug('k=' + str(k))
            self.update(k)
        else:
            #if we cross target position, which is good, we try to minimize error
            if (self.positionT1 > self.targetPosition and self.positionAtZeroVelocity < self.targetPosition - self.threshold ) or (self.positionT1 < self.targetPosition and self.positionAtZeroVelocity > self.targetPosition + self.threshold ) :
                k=0
                if self.positionT1 > self.targetPosition:
                    k = 1
                if self.positionT1 < self.targetPosition:
                    k = -1
                logging.debug('k=' + str(k))
                self.update(k)
            else:
                if (self.positionT1 > self.targetPosition and self.positionAtZeroVelocity > self.targetPosition + self.threshold ) or (self.positionT1 < self.targetPosition and self.positionAtZeroVelocity < self.targetPosition - self.threshold ) :
                    k = 0
                    if self.positionT1 > self.targetPosition:
                        k = -1
                    if self.positionT1 < self.targetPosition:
                        k = 1
                    logging.debug('k=' + str(k))
                    self.update(k)

