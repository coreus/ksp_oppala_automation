import time
import logging
from helper import Helper
from derivative import Derivative
from flightplan import FlightPlan
from krpc.services.spacecenter import DockingPortState

class Rendezvous:
    def __init__(self,flight:FlightPlan,target:FlightPlan):
        self.flight = flight   #tanker
        self.target = target
        
    def execute(self):
        self.flight.vessel.control.rcs = True
        while not (self.distanceForecast() < 50 and self.speed() < 1):
            while self.speed() > 1:
                self.flight.conn.space_center.warp_to(self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit))
                self.nullifyRelativeSpeed()
                if self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit) - self.flight.ut() < 10:
                    break
            if self.distanceForecast() > 150:
                maxSpeed = 6
            else:
                maxSpeed = 1
            self.follow(maxSpeed)
        
        time.sleep(2)
        
        self.dock()
        time.sleep(10)
        self.transfer()
        time.sleep(1)
        self.undock()
        

    def identifyTanks(self):
        self.TankerLox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[0].amount
        self.TankerPropelant = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount
        self.TargetLox = self.target.vessel.resources.with_resource("Oxidizer")[0].amount
        self.TargetPropelant = self.target.vessel.resources.with_resource("LiquidFuel")[0].amount

    def transfer(self, left = 400):
        count = 0
        if self.TankerLox != self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[0].amount:
            count = 1
        
        self.TankerLox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[(0 + count) % 2]
        self.TargetLox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[(1 + count) % 2]

        count = 0
        if self.TankerPropelant != self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount:
            count = 1
        
        self.TankerPropelant = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[(0 + count) % 2]
        self.TargetPropelant = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[(1 + count) % 2]

        self.flight.conn.space_center.ResourceTransfer.start(self.TankerLox.part,self.TargetLox.part,"Oxidizer",self.TankerLox.amount-left)
        self.flight.conn.space_center.ResourceTransfer.start(self.TankerPropelant.part,self.TargetPropelant.part,"LiquidFuel",self.TankerPropelant.amount-left)

    def facing(self):

        logging.info("Facing tanker")
        self.target.control()
        self.target.vessel.control.rcs = True
        self.target.conn.space_center.target_vessel = self.flight.vessel
        self.target.conn.space_center.target_docking_port = self.flight.vessel.parts.docking_ports[0]
        self.target.setSAS(mode=self.target.vessel.control.sas_mode.target)
        time.sleep(5)
        self.target.activateRCS(False)

        logging.info("Facing vessel")
        self.flight.control()
        self.flight.vessel.control.rcs = True
        self.flight.conn.space_center.target_vessel = self.target.vessel
        self.flight.conn.space_center.target_docking_port = self.target.vessel.parts.docking_ports[0]
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.target)
        time.sleep(5)
        

    def dock(self):
        while len(self.flight.conn.space_center.active_vessel.parts.docking_ports) == 1:
            try:
                self.facing()
                self.flight.control()
                self.flight.activateRCS()
                self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.target)
                self.flight.vessel.control.forward = 1
                while self.speed() < 0.5:
                    time.sleep(0.1)
                self.flight.vessel.control.forward = 0
                dDistance = Derivative(self.distance, 0.01)
                
                self.identifyTanks()
                while dDistance.getDelta() < 0:
                    if self.distance()<5:
                        self.flight.vessel.control.forward = -1
                        while self.speed() > 0.2:
                            time.sleep(0.1)
                        self.facing()
                        self.flight.control()
                        self.flight.vessel.control.forward = 0
                        break
                    time.sleep(0.1)
            except:
                continue
            
    def undock(self):
        for docking_port in self.flight.conn.space_center.active_vessel.parts.modules_with_name("ModuleDockingNode"):
            if docking_port.has_event('Désamarrer'):
                docking_port.trigger_event('Désamarrer')
                break
        self.flight.conn.space_center.clear_target()

    def speed(self):
        return Helper.speed(self.flight.vessel,self.target.vessel)
    
    def distance(self):
        return Helper.dist(self.flight.vessel,self.target.vessel)

    def distanceForecast(self):
        return self.flight.vessel.orbit.distance_at_closest_approach(self.target.vessel.orbit)

    def nullifyRelativeSpeed(self):
        self.flight.conn.space_center.target_vessel = self.target.vessel
        """
        Executes the rendezvous maneuver by warping to the time of closest approach
        between the active vessel and the target vessel.

        This method uses the KSP (Kerbal Space Program) API to control the space center
        and warp to the calculated time of closest approach.

        Raises:
            SomeException: Description of the exception that might be raised.
        """
        closest_approach_distance = self.flight.vessel.orbit.distance_at_closest_approach(self.target.vessel.orbit)
        if closest_approach_distance < 30:
            warp_time = self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit) - 15
        else:
            warp_time = self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit)
        self.flight.conn.space_center.warp_to(warp_time)
        #self.flight.conn.space_center.warp_to(self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit))
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.retrograde)
        time.sleep(5)
        dspeed = Derivative(self.speed, 0.01)
        self.flight.vessel.control.throttle = 0.1
        while dspeed.getDelta() < 0:
            time.sleep(0.01)
            if self.speed() < 3:
                self.flight.vessel.control.throttle = 0.01
            else:
                time.sleep(0.09)
        self.flight.vessel.control.throttle = 0

    def follow(self, maxSpeed=6):
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.target)
        time.sleep(5)
        self.flight.vessel.control.throttle = 0.1
        
        while self.speed() < maxSpeed:
            time.sleep(0.1)
        self.flight.vessel.control.throttle = 0
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.retrograde)
        time.sleep(0.5)
        self.flight.conn.space_center.warp_to(self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(self.target.vessel.orbit))
        self.nullifyRelativeSpeed()
    