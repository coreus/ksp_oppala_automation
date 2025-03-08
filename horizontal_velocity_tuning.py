from abstract_tuning import AbstractTuning
import logging
class HorizontalVelocityTuning(AbstractTuning):
    def __init__(self,getSpeed,targetAngle,setSAS,setControlPitch,getPitch,autoPilot):
        super().__init__(getSpeed)
        self.setControlPitch = setControlPitch
        self.getPitchAngle = getPitch
        self.autoPilot = autoPilot
        self.targetAngle = targetAngle
        self.setSAS = setSAS
        self.reset()

    def update(self,k):
        
        logging.debug(self.getPitchAngle())
        logging.debug(self.targetAngle)
        logging.debug(self.positionT0)
        self.setControlPitch(-1 * k)
        return 
        if k != self.firstK:
            self.autoPilot.engage()
            self.autoPilot.target_direction = (1, 0, 0)
            return
        
        if abs(self.positionT0) < 1:
            self.autoPilot.engage()
            self.autoPilot.target_direction = (1, 0, 0)
            return
        
        if self.getPitchAngle() > self.targetAngle:
            self.autoPilot.disengage()
            self.setControlPitch(-1 * k)
        else:
            self.setSAS()
            return
        
            
        
