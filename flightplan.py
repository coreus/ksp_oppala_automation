import krpc
import logging
import math
import time
import numpy as np
import csv
import krpc.services.spacecenter
from telemetry import Telemetry
from derivative import Derivative
from dataset import Dataset
from flaptuning import FlapTuningV2


#from rendezvous import Rendezvous

class FlightPlan:
    def __init__(self,conn: krpc.Client, vessel: krpc.services.spacecenter.Vessel = None ) -> None:
        logging.basicConfig(level=logging.INFO)
        self.conn = conn
        self.flightId = time.time()
        if vessel is None:
            self.vessel = self.conn.space_center.active_vessel
        else:
            self.vessel = vessel
        self.flight = self.vessel.flight()
        self.telemetry = Telemetry(self.conn,self.vessel)
        self.altitude = self.telemetry.streamAltitude()
        self.apoapsis = self.telemetry.streamApoapsis()
        self.fuel = self.telemetry.streamFuel(self.vessel.control.current_stage - 1)
        self.speed = self.telemetry.streamSpeed()
        self.verticalSpeed = self.telemetry.streamVerticalSpeed()
        self.deltaVspeed = Derivative(self.verticalSpeed)
        self.pitch = self.telemetry.streamPitch()
        self.deltaPitch = Derivative(self.pitch)
        self.ut = self.telemetry.streamUT()
        self.vesselHeight = self.altitude()

        self.velocity = self.telemetry.streamVelocity(self.conn.space_center.ReferenceFrame.create_hybrid(
                                                                                            position=self.vessel.orbit.body.reference_frame,
                                                                                            rotation=self.vessel.surface_reference_frame))

        self.position = {'lat':self.telemetry.streamLatitude(),'long':self.telemetry.streamLongitude()}
        self.padCoordinates = {'lat':self.position['lat'](),'long':self.position['long']()}
        
        self.padPosition = self.vessel.position(self.vessel.orbit.body.reference_frame)
        self.padReferenceFrame = self.conn.space_center.ReferenceFrame.create_relative(self.vessel.orbit.body.reference_frame,position=self.padPosition)
        self.horizontalSpeed = self.telemetry.streamHorizontalSpeed(self.padReferenceFrame)
        self.deltaDistPad = Derivative(self.distanceToPad)
        self.backFlaps = self.vessel.parts.with_title('Ailette delta Prestige')
        self.frontFlaps = self.vessel.parts.with_title('Petite aile delta')
        logging.info(self.padCoordinates)
        self.logLanding = False
        self.neuralLand = False
        self.deltaAngle = 0
        self.identifyFlaps()

    def setLoggingLanding(self):
        self.logLanding = True
        self.landingDataset = Dataset("landing.csv")

    def setNeuralLanding(self):
        self.neuralLand = True
 #       self.neuralLanding = NeuralLanding("landing.v3.model")

    def control(self):
        self.conn.space_center.active_vessel = self.vessel

    def setSAS(self,mode = None):
        self.vessel.auto_pilot.disengage()
        self.vessel.control.sas = True
        time.sleep(0.1)
        self.vessel.control.sas_mode = (self.vessel.control.sas_mode.stability_assist if mode == None else mode)
        
    
    def testLaunch(self,shutdownHeight=3000) -> None:
        self.setFlap([90,90])
        time.sleep(1)
        self.vessel.control.rcs = True
        self.vessel.control.throttle = 1.0
        
        self.setSAS()
        
        self.separateStage()
        self.vessel.parts.engines
        
        while self.altitude()<300:
            time.sleep(1.5)
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
        
        stageSeparation = False
        while True:
            logging.debug("distance to pad : " + str(self.distanceToPad()))

            if self.altitude()> 1000 and not stageSeparation:
                self.separateStage()
                self.vessel.control.activate_next_stage()
                stageSeparation = True
                start = self.ut()
                startMass = self.vessel.mass
            if self.altitude()> shutdownHeight:
                self.vessel.control.throttle = 0
                end = self.ut()
                endMass = self.vessel.mass
                fuelMassConsumption = (startMass - endMass) / (end - start)
                logging.info("fuel mass consumption : " + str(fuelMassConsumption))
                
                break
            time.sleep(1)
        yaw = self.yawToPoint(self.padCoordinates['lat'],self.padCoordinates['long'])
        self.vessel.auto_pilot.target_heading = yaw
        self.vessel.auto_pilot.target_roll=0
        time.sleep(2)
        self.vessel.auto_pilot.wait()
        self.vessel.control.rcs = False
        while self.verticalSpeed()>0:
            time.sleep(0.5)
        logging.info("On éteint les moteurs!")
        self.vessel.control.throttle = 0

    def launchToOrbit(self,orbitalAltitude,gravityTurnStart,gravityTurnEnd):
        self.setFlap([90,90])
        orbitalVelocity = self.calculateVelocityForOrbit(orbitalAltitude)
        logging.info('calculate orbital velocity to reach ' + str(orbitalVelocity) + ' m/s')
        self.vessel.control.sas = False
        #self.vessel.control.rcs = True
        self.vessel.control.throttle = 1.0

        #logging.info("3")
        #time.sleep(1)
        #logging.info("2")
        #time.sleep(1)
        #logging.info("1")
        #time.sleep(1)
        logging.info("lift off")
        
        startTime = self.ut()
        
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_pitch_and_heading(90, 90)
        turnAngle = 0
        self.separateStage()
        #apoapsis = self.apoapsis()
        #hSpeed = self.horizontalSpeed()
        while True:
            if self.fuel() < 10:
                self.vessel.control.rcs = False
                self.separateStage()
            
            altitude = self.altitude()


            if turnAngle < 90 and altitude > gravityTurnStart:
                logging.info('processing gravity turn')
                frac = ((altitude - gravityTurnStart) /
                        (gravityTurnEnd - gravityTurnStart))
                newTurnAngle = frac * 90
                if abs(newTurnAngle - turnAngle) > 0.5:
                    turnAngle = newTurnAngle
                    logging.info('set turn angle ' + str(turnAngle))
                    self.vessel.auto_pilot.target_pitch_and_heading(90-turnAngle, 90)
            
            if turnAngle< 90 and self.vessel.control.throttle == 1 and self.apoapsis() > orbitalAltitude*0.9:
                logging.info('Approaching target apoapsis, throttle engines')
                self.vessel.control.throttle = 0.25
            if self.vessel.control.throttle> 0 and  turnAngle< 90 and self.apoapsis() >= orbitalAltitude:
                logging.info('Shutdown engines')
                self.vessel.control.throttle = 0

            if turnAngle>=90:
                logging.info('Shutdown engines')
                self.vessel.control.throttle = 0
                break
            
            time.sleep(1)
        
        logging.info('waits to be out ot atmosphere')
        while self.altitude() < self.vessel.orbit.body.atmosphere_depth:
            time.sleep(1)



        mu = self.vessel.orbit.body.gravitational_parameter
        r = self.vessel.orbit.apoapsis
        a1 = self.vessel.orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        deltaV = v2 - v1
        node = self.vessel.control.add_node(
            self.ut() + self.vessel.orbit.time_to_apoapsis, prograde=deltaV)

        # Calculate burn time (using rocket equation)
        F = self.vessel.max_thrust
        Isp = self.vessel.specific_impulse * self.vessel.orbit.body.surface_gravity
        m0 = self.vessel.mass
        m1 = m0 / math.exp(deltaV/Isp)
        flowRate = F / Isp
        burnTime = (m0 - m1) / flowRate

        # Orientate ship
        logging.info('Orientating ship for circularization burn')
        self.vessel.auto_pilot.reference_frame = node.reference_frame
        self.vessel.auto_pilot.target_direction = (0, 1, 0)
        self.vessel.auto_pilot.wait()

        # Wait until burn
        logging.info('Waiting until circularization burn')
        burnUt = self.ut() + self.vessel.orbit.time_to_apoapsis - (burnTime/2.)
        leadTtime = 5
        self.conn.space_center.warp_to(burnUt - leadTtime)


        # Execute burn
        logging.info('Ready to execute burn')
        timeToApoapsis = self.telemetry.streamTimeToApoapsis()
        while timeToApoapsis() - (burnTime/2.) > 0:
            pass
        logging.info('Executing burn')
        self.vessel.control.throttle = 1.0
        time.sleep(burnTime - 0.1)
        logging.info('Fine tuning')
        self.vessel.control.throttle = 0.01
        remainingBurn = self.conn.add_stream(node.remaining_burn_vector, node.reference_frame)
        deltaBurn =Derivative(remainingBurn,0.001)
        self.vessel.control.throttle = 0.2
        while remainingBurn()[1] > 0.1 and deltaBurn.getDelta(1)< 0:
            #logging.info(remainingBurn()[1])
            pass
        self.vessel.control.throttle = 0.0
        node.remove()
        endTime = self.ut()
        longitude = self.position['long']()
        duration = endTime - startTime
        logging.info('Orbit achieved in ' + str(duration) + ' s with longitude ' + str(longitude))
        self.openDockingPort()
        #self.deploySolarPanels()

        return self.fuel()                
    

    def deploySolarPanels(self, state = True):
        for panel in self.vessel.parts.solar_panels:
            panel.deployed = state
    
    def deployAntenna(self, state = True):
        try:
            self.vessel.parts.with_name("mediumDishAntenna")[0].antenna.deployed = state
        except IndexError:
            logging.warning("No mediumDishAntenna found, cannot deploy antenna")
        try:
            self.vessel.parts.with_name("HighGainAntenna")[0].antenna.deployed = state
        except IndexError:
            logging.warning("No HighGainAntenna found, cannot deploy antenna")
        time.sleep(1)
        
    def openDockingPort(self, state = True):
        self.vessel.parts.docking_ports[0].shielded = not state

    def reentry(self,file):
        self.deploySolarPanels(False)
        self.openDockingPort(False)
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
        #self.vessel.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        
        self.vessel.auto_pilot.target_roll = 0
        self.vessel.control.rcs = False
        flight = self.vessel.flight()
        tuner = FlapTuningV2(self.getFrontFlapAngle,self.getBackFlapAngle,self.setFrontFlap, self.setBackFlap,self.getPitch)
        tuner.setReverse(False)
        tuner.setStep(0.5)
        tuner.setThreshold(5)
        with open(file,"r") as csvfile:
            desorbitreader = csv.reader(csvfile)
            for row in desorbitreader:
                (angle, bf,ff,speed,mode) = row
                if bf != "":
                    self.setBackFlap(int(bf))
                if ff != "":
                    self.setFrontFlap(int(ff))
                if mode == "reentry":
                    tuner.setFrontState(True)
                    tuner.setBackState(True)
                if mode == "dive":
                    tuner.setFrontState(True)
                    tuner.setBackState(False)
                
                #self.vessel.auto_pilot.target_pitch = float(angle)
                self.vessel.auto_pilot.target_pitch_and_heading(float(angle),self.yawToPoint(self.padCoordinates["lat"],self.padCoordinates["long"]))
                
                tuner.setTargetPosition(float(angle) + self.getProgradAngle())
                #aligned = False
                while self.horizontalSpeed() > float(speed) or float(angle) + self.getProgradAngle()<5:
                    if tuner.reverse == False and flight.mach < 1.2:
                        tuner.setReverse(True)
                    
                    #tuning is useless without atmosphere
                    if flight.atmosphere_density > 0.001:
                        tuner.tune()
                        if abs(tuner.targetPosition - float(angle) - self.getProgradAngle()) > 1:
                            tuner.setTargetPosition(float(angle) + self.getProgradAngle())
                    force =  math.sqrt(pow(flight.aerodynamic_force[0],2) + pow(flight.aerodynamic_force[1],2) + pow(flight.aerodynamic_force[2],2))
                    logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()) + " tangage : " + str(flight.pitch) + " atmo density : " + str(flight.atmosphere_density) + " altitude : " + str(self.altitude()) + " mach: " + str(flight.mach) + " dynamic pressure: " + str(flight.dynamic_pressure) + " aerodynamic force: " + str(force) + " aoa: " + str(flight.angle_of_attack) + " prograde: " + str(self.getProgradAngle()))
                    time.sleep(0.1)

    def landAtKSC(self):
        startAt = -129.4
        startAt = -129.9
        
        distancePerDegree = (self.vessel.orbit.body.equatorial_radius + self.altitude()) * math.pi * 2 / 360
        initialLongitude = self.telemetry.streamLongitude()()

        if initialLongitude > startAt:
            distance = (360 - (initialLongitude - startAt)) * distancePerDegree
        else:
            distance = (startAt - initialLongitude) * distancePerDegree
        
        duration = distance / self.speed()
        self.conn.space_center.warp_to(self.ut() + duration - 10)
        while abs(self.telemetry.streamLongitude()()- startAt) > 1:
            logging.debug("long : " + str(self.telemetry.streamLongitude()()))
            time.sleep(1)

    def desorbit(self,burnDuration=5):
        self.vessel.control.throttle = 0
        self.conn.space_center.clear_target()
        self.openDockingPort(False)
        self.activateRCS(False)
        self.deploySolarPanels(False)
        self.vessel.control.sas = False
        self.vessel.auto_pilot.engage()
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.reference_frame = self.vessel.surface_velocity_reference_frame
        self.vessel.auto_pilot.target_direction = (0, -1, 0)
        time.sleep(1)
        self.vessel.auto_pilot.wait()
        
        position = {'lat':self.telemetry.streamLatitude(),'long':self.telemetry.streamLongitude()}
        logging.info('lat: ' + str(position['lat']()) + ' long: ' + str(position['long']()))
        self.vessel.control.throttle = 1
        logging.info("burn duration : " + str(burnDuration) + " s")
        logging.info(str(self.pinpointCurrentLandingZone()))
        padLong = -74.55773266001711
        while self.pinpointCurrentLandingZone() == False or (self.pinpointCurrentLandingZone()['long']  - padLong) > (7.5 - (self.altitude() - 80000) / 10000):
            logging.info(str(self.pinpointCurrentLandingZone()))
            time.sleep(0.01)
        logging.info(str(self.pinpointCurrentLandingZone()['long']))
        logging.info(str(self.padCoordinates['long']))
        #time.sleep(burnDuration)
        self.vessel.control.throttle = 0
        #self.curveFlaps()
        self.setBackFlap(40)
        self.setFrontFlap(40)
        
        self.vessel.auto_pilot.target_roll = 0
        self.vessel.auto_pilot.target_direction = (1, 1, 0)
        self.vessel.auto_pilot.wait()
        self.vessel.control.rcs = False

        while self.altitude() > 51000:
            time.sleep(0.5)
        
    
    def prelanding(self, angle, prograde):
        self.vessel.auto_pilot.engage()
        flight = self.vessel.flight()
        self.vessel.control.rcs = True
        logging.debug("backflap angle : " + str(angle) + " prograde vector size:" + str(prograde))
        self.setBackFlap(angle)
        
        while self.horizontalSpeed() > 150:
            logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + 'vertical speed : '  + str(self.verticalSpeed()) + " tangage : " + str(flight.pitch))
            time.sleep(0.5)

    def separateStage(self):
        self.vessel.control.activate_next_stage()
        self.fuel = self.telemetry.streamFuel(self.vessel.control.current_stage - 1)

    def calculateVelocityForOrbit(self,altitude) -> int:
        return math.sqrt(self.vessel.orbit.body.gravitational_parameter / (self.vessel.orbit.body.equatorial_radius + altitude))

    
    def yawToPoint(self, lat,long)-> int:
        #radius = self.vessel.orbit.body.equatorial_radius
        
        vesselLat=self.position['lat']()
        vesselLong=self.position['long']()
        #haversine formula

        delta_phi = math.log( math.tan(math.radians(vesselLat) / 2 + math.pi / 4 ) / math.tan(math.radians(lat) / 2 + math.pi / 4 ))
        delta_long = abs(math.radians(long) - math.radians(vesselLong))
        return math.atan2(delta_long, delta_phi)


    def distanceToPoint(self,lat,long) -> float:
        radius = self.vessel.orbit.body.equatorial_radius
        vesselLat=self.position['lat']()
        vesselLong=self.position['long']()
        return radius * math.acos(math.sin(math.radians(vesselLat)) * math.sin(math.radians(lat)) + math.cos(math.radians(vesselLat)) * math.cos(math.radians(lat)) * math.cos(math.radians(vesselLong)-math.radians(long)))


    def targetDirection(self, target) -> int :
        angle = target - self.vessel.flight().heading
        if abs(angle) > 180 :
            return (360 - abs(angle)) * angle / abs(angle)
        else :
            return angle
        

    def curveFlaps(self) -> None:
        logging.info("curve flaps")
        #back flaps
        self.vessel.parts.robotic_hinges[0].target_angle = 20
        self.vessel.parts.robotic_hinges[1].target_angle = 20

        #front flaps
        self.vessel.parts.robotic_hinges[2].target_angle = 20
        self.vessel.parts.robotic_hinges[3].target_angle = 20

    def logDynamicPressure(self) -> None:
        logging.debug("dynamic pressure - back flaps :" + str(self.getBackDynamicPressure()))
        logging.debug("dynamic pressure - front flaps :" + str(self.getFrontDynamicPressure()))

    def turnLeft(self) -> None:
        #front flaps
        self.vessel.parts.robotic_hinges[2].target_angle = self.vessel.parts.robotic_hinges[2].target_angle - 3
        
    def turnRight(self) -> None:
        #front flaps
        self.vessel.parts.robotic_hinges[3].target_angle = self.vessel.parts.robotic_hinges[3].target_angle - 3

    def getFrontFlapAngle(self) -> int:
        return self.vessel.parts.robotic_hinges[2].target_angle
    
    def getBackFlapAngle(self) -> int:
        return self.vessel.parts.robotic_hinges[0].target_angle

    def fixCurve(curve) -> float:
        if curve > 90:
            return 90
        if curve < 20:
            return 20
        return curve
    
    def setFrontFlap(self,curve) -> None:
        self.vessel.parts.robotic_hinges[2].target_angle = FlightPlan.fixCurve(curve)
        self.vessel.parts.robotic_hinges[3].target_angle = FlightPlan.fixCurve(curve)

    def setBackFlap(self,curve) -> None:
        self.vessel.parts.robotic_hinges[0].target_angle = FlightPlan.fixCurve(curve)
        self.vessel.parts.robotic_hinges[1].target_angle = FlightPlan.fixCurve(curve)

    def setFlap(self,curve):
        self.setBackFlap(curve[0])
        self.setFrontFlap(curve[1])
    
    def setControlPitch(self,pitch):
        logging.debug("control pitch : " + str(pitch))
        self.vessel.control.pitch = pitch

    def getHorizontalVelocity(self):
        return self.velocity()[1]

    def spin(self, direction) -> None:
        if direction == "left":
            self.vessel.parts.robotic_hinges[0].target_angle = 120
            self.vessel.parts.robotic_hinges[1].target_angle = 30
            self.vessel.parts.robotic_hinges[2].target_angle = 120
            self.vessel.parts.robotic_hinges[3].target_angle = 30
        else :
            self.vessel.parts.robotic_hinges[0].target_angle = 30
            self.vessel.parts.robotic_hinges[1].target_angle = 120
            self.vessel.parts.robotic_hinges[2].target_angle = 30
            self.vessel.parts.robotic_hinges[3].target_angle = 120


    def bellyFlop(self):
        self.vessel.control.throttle = 0.3
        self.vessel.control.rcs = True
        
        self.setFrontFlap(90)
        self.setBackFlap(20)
        self.setSAS(mode=self.vessel.control.sas_mode.retrograde)

        while abs(self.flight.pitch - 90) > 3:
            time.sleep(0.01)
        self.setFrontFlap(20)
        
        self.vessel.control.throttle = 1
        

    def identifyFlaps(self) -> None:
        front = ["delta.small"]
        back = ["winglet3", "wingConnector4"]
        self.front = []
        self.back = []
        for part in self.vessel.parts.all:
            if part.name in front:
                self.front.append(part)
            if part.name in back:
                self.back.append(part)
        
    def getFrontDynamicPressure(self) -> float:
        pressure = 0
        for wing in self.front:
            pressure = pressure + wing.dynamic_pressure
        return pressure

    def getBackDynamicPressure(self) -> float:
        pressure = 0
        for wing in self.back:
            pressure = pressure + wing.dynamic_pressure
        return pressure
    
    def getPitch(self) -> float:
        return self.flight.pitch
    
    def getProgradAngle(self) -> float:
        return math.asin(np.dot((1,0,0),self.flight.prograde))  * (180.0/math.pi)
        
    def initFlaps(self) -> None:
        logging.info("init flaps")

        #back flaps
        self.vessel.parts.robotic_hinges[0].target_angle = 90
        self.vessel.parts.robotic_hinges[1].target_angle = 90

        #front flaps
        self.vessel.parts.robotic_hinges[2].target_angle = 90
        self.vessel.parts.robotic_hinges[3].target_angle = 90

    def deployLegs(self) -> None:
        for leg in self.vessel.parts.legs:
            logging.info("deploy legs")
            leg.deployed = True


    def activateRCS(self,active = True) -> None:
        for rcs in self.vessel.parts.rcs:
            if rcs.part.name == "RCSBlock.v2":
                rcs.enabled = active
       
    def distanceToPad(self) -> float:
        return self.distanceToPoint(self.padCoordinates['lat'],self.padCoordinates['long'])

    def prepareLanding(self) -> None:
        logging.debug("prepare landing")
        self.setFrontFlap(30)
        self.setBackFlap(90)
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        flight = self.vessel.flight()
        
        self.vessel.auto_pilot.target_roll = 0
        self.vessel.auto_pilot.target_pitch = 0

        while flight.pitch > 25:
            time.sleep(0.1)

    def reachSpeed(self,targetSpeed) -> None:
        #reach target speed
        vSpeed = self.verticalSpeed()
        self.vessel.control.throttle = 1

        if targetSpeed > vSpeed:
            while self.verticalSpeed()<targetSpeed:
                time.sleep(0.01)
        else:
            while self.verticalSpeed()>targetSpeed:
                time.sleep(0.01)

        

    def maintainSpeed(self) -> None:
        self.vessel.control.throttle = self.vessel.mass * self.vessel.orbit.body.surface_gravity / self.vessel.max_thrust
    
    def targetAngle(self,marginheight) -> float:
        return math.atan(self.horizontalSpeed() * abs(self.verticalSpeed()) / (self.vessel.orbit.body.surface_gravity * (self.altitude() - marginheight))) * 180 / math.pi
    

    def cancelHorizontalVelocity(self):
        
        if not hasattr(self,'velocity'):
            self.vessel.control.rcs = True
            self.velocity = self.telemetry.streamVelocity(self.conn.space_center.ReferenceFrame.create_hybrid(
                                                                                            position=self.vessel.orbit.body.reference_frame,
                                                                                            rotation=self.vessel.surface_reference_frame))
        logging.debug(self.velocity())
        if self.velocity()[1]>1:
            self.vessel.control.up = 1
            #self.vessel.control.pitch = 0.5
            logging.debug("up -1")
        if self.velocity()[1]<-1:
            logging.debug("up +1")
            self.vessel.control.up = -1
            #self.vessel.control.pitch = -0.5
            
        logging.debug(self.vessel.control.pitch)
        if abs(self.velocity()[1])<1:
            self.vessel.control.up = 0
            #self.vessel.control.pitch = 0
            
        if self.velocity()[2]>1:
            logging.debug("right -1")
            self.vessel.control.right = -1
        if self.velocity()[2]<-1:
            logging.debug("right +1")
            self.vessel.control.right = 1
        if abs(self.velocity()[2])<1:
            self.vessel.control.right = 0

    def landing(self, minAltitude = 200) -> None:
        throttle = 1.0
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.reference_frame = self.vessel.surface_reference_frame
        flight = self.vessel.flight()
        self.vessel.control.rcs = False
        self.vessel.auto_pilot.target_roll = 0
        self.vessel.auto_pilot.target_pitch = 0
        
        tuner = FlapTuningV2(self.getFrontFlapAngle,self.getBackFlapAngle,self.setFrontFlap, self.setBackFlap,self.getPitch)
        tuner.setTargetPosition(0)
        tuner.setThreshold(1)
        tuner.setReverse(True)
        
        baseMass = self.vessel.mass
        y_acceleration = self.vessel.max_thrust * throttle / baseMass
        
        while True:
            logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()) + " tangage : " + str(flight.pitch) + " atmo density : " + str(flight.atmosphere_density))
            time.sleep(0.1)
            tuner.tune()
            logging.debug('front flap : ' + str(self.getFrontFlapAngle()) + ' back flap : '  + str(self.getBackFlapAngle()))
            altitude = self.altitude() - 2.5 * self.speed()
            squareSpeed = 2 * altitude * (y_acceleration - self.vessel.orbit.body.surface_gravity)
            if squareSpeed < 0:
                break
            if (self.speed() > math.sqrt(squareSpeed) and altitude < 10000) or altitude < minAltitude:
                break
        
        
        logging.debug('start belly flop altitude : ' +  str(self.altitude()) +' horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()))
        
        self.bellyFlop()
        logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()))
        self.reachSpeed(-20)
        
        self.deployLegs()
        
        logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()) + ' prograde : ' + str(self.getProgradAngle()))
        
        
        while self.altitude() > 30:
            self.maintainSpeed()
            logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()) + ' prograde : ' + str(self.getProgradAngle()))
            logging.debug("altitude:" + str(self.altitude()))
            time.sleep(0.1)
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_direction = (1, 0, 0)
        
        while self.altitude() > 20:
            self.maintainSpeed()
            time.sleep(0.1)
        self.reachSpeed(-5)
        
        logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()) + ' prograde : ' + str(self.getProgradAngle()))
        position = {'lat':self.telemetry.streamLatitude(),'long':self.telemetry.streamLongitude()}
        logging.info('lat: ' + str(position['lat']()) + ' long: ' + str(position['long']()))
        while self.altitude() > 7:
            self.maintainSpeed()
            time.sleep(0.1)
            
            logging.debug('horizontal speed : ' + str(self.horizontalSpeed()) + ' vertical speed : '  + str(self.verticalSpeed()))
            logging.debug("altitude:" + str(self.altitude()))
            
        logging.info("engines shutdown")
        self.vessel.control.throttle = 0
        time.sleep(3)
        self.vessel.control.rcs = False
        return
        
        
        if self.logLanding:
            self.landingDataset.close()
        
    def touchDown(self):
        logging.debug("touchdown")
        self.vessel.control.rcs = False
    

    #def rdv(self,target):
    #    rdv = Rendezvous(self,target)
    #    rdv.execute()
    
    def transferErgol(self, tanker, target_ship, amount):
        # Ensure both vessels are docked
        if not tanker.docking_ports[0].has_docked_part or not target_ship.docking_ports[0].has_docked_part:
            logging.error("Both vessels must be docked to transfer ergol.")
            return
        self.conn.space_center.target_docking_port
        # Get the resource to transfer
        resource = 'LiquidFuel'

        # Get the amount of resource in the tanker
        available_amount = tanker.resources.amount(resource)
        if available_amount < amount:
            logging.warning(f"Not enough {resource} in the tanker. Available: {available_amount}, Requested: {amount}")
            amount = available_amount

        # Transfer the resource
        tanker.resources.transfer(target_ship.resources, resource, amount)
        logging.info(f"Transferred {amount} units of {resource} from tanker to target ship.")
    
    def pinpointCurrentLandingZone(self):
        if self.vessel.orbit.periapsis_altitude < 0:
            body = self.vessel.orbit.body
            # Recherche dichotomique du temps d'impact
            t0 = self.conn.space_center.ut
            t1 = t0 + 900
            
            for _ in range(20):  # 20 itérations suffisent souvent
                tm = (t0 + t1) / 2
                pos = self.vessel.orbit.position_at(tm, body.reference_frame)
                alt = body.altitude_at_position(pos,body.reference_frame)
                if alt < 0:
                    t1 = tm
                else:
                    t0 = tm

            impact_time = (t0 + t1) / 2
            impact_pos = self.vessel.orbit.position_at(impact_time, body.reference_frame)
            lat, lon, alt = body.latitude_at_position(impact_pos,body.reference_frame),body.longitude_at_position(impact_pos,body.reference_frame), body.altitude_at_position(impact_pos,body.reference_frame)

            print(f"Impact prévu à t+{impact_time - self.conn.space_center.ut:.1f}s")
            print(f"Latitude: {lat:.4f}, Longitude: {lon:.4f}")
            return {'lat': lat, 'long': lon, 'altitude': alt, 'time': impact_time}
        return False
    
    def recoverShip(self):
        logging.info("recover ship")
        if self.vessel.recoverable:
            self.vessel.recover()
            logging.info("ship recovered")
    
    def alignWithPad(self, tolerance_long=0.05, tolerance_lat=0.05, max_time=120):
        """
        Ajuste la trajectoire pour passer au-dessus du pas de tir (self.padCoordinates).
        Corrige le heading jusqu'à ce que la longitude et la latitude soient proches du pad.
        """
        padLat = self.padCoordinates['lat']
        padLong = self.padCoordinates['long']
        start_time = time.time()
        while True:
            currentLat = self.telemetry.streamLatitude()()
            currentLong = self.telemetry.streamLongitude()()
            d_long = abs(currentLong - padLong)
            d_lat = abs(currentLat - padLat)
            if d_long < tolerance_long and d_lat < tolerance_lat:
                logging.info(f"Aligné avec le pas de tir : lat={currentLat:.4f}, long={currentLong:.4f}")
                break
            # Calcul du cap pour viser le pas de tir
            yaw = self.yawToPoint(padLat, padLong)
            self.vessel.auto_pilot.target_pitch_and_heading(self.vessel.auto_pilot.target_pitch, math.degrees(yaw))
            logging.debug(f"Correction heading: {math.degrees(yaw):.2f} | lat: {currentLat:.4f} | long: {currentLong:.4f}")
            time.sleep(0.5)
            if time.time() - start_time > max_time:
                logging.warning("Alignement interrompu (timeout)")
                break

    def landAtKSCSequence(self):
        """
        Aligne la trajectoire sur le pas de tir puis lance la séquence d'atterrissage KSC.
        """
        #self.alignWithPad()
        self.landAtKSC()


    def getBodiesAngle(self, origin_body: krpc.services.spacecenter.CelestialBody, target_body: krpc.services.spacecenter.CelestialBody,) -> float:
        """
        Calcule l'angle entre deux corps célestes à partir de leur position actuelle.
        """
        ut: float = self.conn.space_center.ut
        star = self.conn.space_center.bodies["Sun"]
        #dot_product = sum(a*b for a, b in zip(origin_body.orbit.position_at(ut, star.reference_frame), target_body.orbit.position_at(ut, star.reference_frame)))
        #norm1 = math.sqrt(sum(a*a for a in origin_body.orbit.position_at(ut, star.reference_frame)))
        #norm2 = math.sqrt(sum(b*b for b in target_body.orbit.position_at(ut, star.reference_frame)))

        ## Calcul de l'angle
        #cos_theta = dot_product / (norm1 * norm2)

        pos1 = origin_body.orbit.position_at(ut, star.reference_frame)
        pos2 = target_body.orbit.position_at(ut, star.reference_frame)
        
        dot_product = sum(a * b for a, b in zip(pos1, pos2))
        norm1 = math.sqrt(sum(a * a for a in pos1))
        norm2 = math.sqrt(sum(b * b for b in pos2))
        
        cos_theta = dot_product / (norm1 * norm2)
        theta_rad = math.acos(cos_theta)
        
        return theta_rad

    def waitForHohmannTransfer(self, origin_body: krpc.services.spacecenter.CelestialBody,target_body: krpc.services.spacecenter.CelestialBody) -> None:
        mu: float = self.conn.space_center.bodies["Sun"].gravitational_parameter
        r1: float = origin_body.orbit.semi_major_axis
        r2: float = target_body.orbit.semi_major_axis
        a: float = (r1 + r2) / 2
        logging.info(f"Transfert de Hohmann vers {target_body.name} depuis {origin_body.name}")
        logging.info(f"Distance actuelle : {r1:.2f} m, Distance cible : {r2:.2f} m")
        # Calcul de la durée de transfert
        transfer_time: float = math.pi * math.sqrt((r1 + r2)**3 / (8 * mu))
        logging.info(f"Durée de transfert estimée : {transfer_time / (24 * 3600):.2f} jours")

        # Vitesse angulaire
        n1 = 2 * math.pi / origin_body.orbit.period
        n2 = 2 * math.pi / target_body.orbit.period
        
        phase_angle: float = 180 * (1 - math.sqrt(math.pow(r1,3) / math.pow(r2,3)))
        phase_angle: float = math.pi - n2 * transfer_time
        logging.info(f"Angle de phase optimal pour le transfert : {math.degrees(phase_angle):.2f}°")
        
        

        while round(math.degrees(self.getBodiesAngle(origin_body, target_body))) != round(math.degrees(phase_angle)):

           
            theta_rad = self.getBodiesAngle(origin_body, target_body)
            theta_deg = math.degrees(theta_rad)
            logging.info(f"Angle actuel entre le vaisseau et la cible : {theta_deg:.2f}°")
            
            

            wait_time: float = ((theta_rad - phase_angle)) / (n1 - n2)
            logging.info(f"Temps jusqu'à la fenêtre de transfert : {wait_time / (3600 * 24):.2f}")
            
            # Calcul du temps d'attente jusqu'à la fenêtre optimale
            # L'angle de phase optimal est calculé plus haut
            angle_diff = (theta_rad - phase_angle) % (2 * math.pi)
            if angle_diff < 0:
                angle_diff += 2 * math.pi
            logging.info(f"Différence d'angle pour la fenêtre de transfert : {math.degrees(angle_diff):.2f}°")
            

            
            

            logging.info(origin_body.orbit.period)
            logging.info(target_body.orbit.period)
            logging.info(f"Vitesse angulaire du vaisseau : {n1:.12f} rad/s")
            logging.info(f"Vitesse angulaire de la cible : {n2:.12f} rad/s")
            relative_angular_velocity = n1 - n2
            logging.info(f"Vitesse angulaire relative : {relative_angular_velocity:.12f} rad/s")
            # Temps d'attente jusqu'à la fenêtre
            #wait_time = (angle_diff * 180 / math.pi) / relative_angular_velocity
            
            logging.info(f"Attente de {wait_time:.0f} secondes pour la fenêtre de transfert optimale")
            
            # Avance le temps jusqu'à la fenêtre de transfert
            transfer_ut = self.conn.space_center.ut + wait_time
            #logging.info(self.conn.space_center.warp_factor)
            #self.conn.space_center.rails_warp_factor  = 7
            logging.info(f"Avance le temps à {transfer_ut} UT pour le transfert de Hohmann")
            self.conn.space_center.warp_to(transfer_ut - 60)  # Arrive 1 minute avant pour se préparer

    def executeHohmannTransfer(self, target_body: krpc.services.spacecenter.CelestialBody) -> None:
        """
        Effectue un transfert de Hohmann vers le corps céleste cible (target_body).
        """
        vessel: krpc.services.spacecenter.Vessel = self.vessel
        origin_body: krpc.services.spacecenter.CelestialBody = self.conn.space_center.bodies["Sun"]
        mu: float = origin_body.gravitational_parameter
        r1: float = vessel.orbit.body.orbit.semi_major_axis
        r2: float = target_body.orbit.semi_major_axis
        a: float = (r1 + r2) / 2
        logging.info(f"Transfert de Hohmann vers {target_body.name} depuis {origin_body.name}")
        logging.info(f"Distance actuelle : {r1:.2f} m, Distance cible : {r2:.2f} m")
        # Calcul de la durée de transfert
        transfer_time: float = math.pi * math.sqrt((r1 + r2)**3 / (8 * mu))
        logging.info(f"Durée de transfert estimée : {transfer_time / (24 * 3600):.2f} jours")

        # Vitesse angulaire
        n1 = 2 * math.pi / vessel.orbit.body.orbit.period
        n2 = 2 * math.pi / target_body.orbit.period
        
        
        #transfer_time = math.pi * math.sqrt(((r1 + r2) ** 3) / (8 * mu))
        #phase_angle_target = math.pi - n2 * transfer_time
        #phase_angle_target = phase_angle_target % (2*math.pi)
#
        #print(f"Angle de phase cible (°): {math.degrees(phase_angle_target):.2f}")

        # Calcul du delta-v pour le transfert de Hohmann
        v1: float = math.sqrt(mu / r1)
        v_transfer: float = math.sqrt(mu * (2/r1 - 1/a))
        delta_v1: float = v_transfer - v1
        delta_v2: float = math.sqrt(mu / r2) - v_transfer
        logging.info(f"Delta-v pour le transfert de Hohmann : {delta_v1:.2f} m/s")
        
        
        # Pour une solution robuste, il faudrait calculer l'angle actuel et attendre la bonne fenêtre
        # Calcul de l'angle de phase actuel entre les deux corps
        ut: float = self.conn.space_center.ut
        
        dot_product = sum(a*b for a, b in zip(vessel.orbit.body.orbit.position_at(ut, origin_body.reference_frame), target_body.orbit.position_at(ut, origin_body.reference_frame)))
        norm1 = math.sqrt(sum(a*a for a in vessel.orbit.body.orbit.position_at(ut, origin_body.reference_frame)))
        norm2 = math.sqrt(sum(b*b for b in target_body.orbit.position_at(ut, origin_body.reference_frame)))

        # Calcul de l'angle
        cos_theta = dot_product / (norm1 * norm2)
        theta_rad = math.acos(cos_theta)
        theta_deg = math.degrees(theta_rad)
        logging.info(f"Angle actuel entre le vaisseau et la cible : {theta_deg:.2f}°")
        
        phase_angle: float = 180 * (1 - math.sqrt(math.pow(r1,3) / math.pow(r2,3)))
        phase_angle: float = math.pi - n2 * transfer_time
        logging.info(f"Angle de phase optimal pour le transfert : {math.degrees(phase_angle):.2f}°")

        wait_time: float = ((theta_rad - phase_angle)) / (n1 - n2)
        logging.info(f"Temps jusqu'à la fenêtre de transfert : {wait_time / (3600 * 24):.2f}")
        
        # Calcul du temps d'attente jusqu'à la fenêtre optimale
        # L'angle de phase optimal est calculé plus haut
        angle_diff = (theta_rad - phase_angle) % (2 * math.pi)
        if angle_diff < 0:
            angle_diff += 2 * math.pi
        logging.info(f"Différence d'angle pour la fenêtre de transfert : {math.degrees(angle_diff):.2f}°")
        

        
        

        logging.info(vessel.orbit.body.orbit.period)
        logging.info(target_body.orbit.period)
        logging.info(f"Vitesse angulaire du vaisseau : {n1:.12f} rad/s")
        logging.info(f"Vitesse angulaire de la cible : {n2:.12f} rad/s")
        relative_angular_velocity = n1 - n2
        logging.info(f"Vitesse angulaire relative : {relative_angular_velocity:.12f} rad/s")
        # Temps d'attente jusqu'à la fenêtre
        #wait_time = (angle_diff * 180 / math.pi) / relative_angular_velocity
        
        logging.info(f"Attente de {wait_time:.0f} secondes pour la fenêtre de transfert optimale")
        
        # Avance le temps jusqu'à la fenêtre de transfert
        transfer_ut = self.conn.space_center.ut + wait_time
        #logging.info(self.conn.space_center.warp_factor)
        #self.conn.space_center.rails_warp_factor  = 7
        self.conn.space_center.warp_to(transfer_ut - 60)  # Arrive 1 minute avant pour se préparer
        
        # Crée un nœud de manœuvre pour le transfert
        ut: float = self.conn.space_center.ut
        node: krpc.services.spacecenter.Node = vessel.control.add_node(ut + 60, prograde=delta_v1)
        
        # Calcul du temps de burn
        F: float = vessel.max_thrust
        Isp: float = vessel.specific_impulse * origin_body.surface_gravity
        m0: float = vessel.mass
        m1: float = m0 / math.exp(delta_v1/Isp)
        flowRate: float = F / Isp
        burnTime: float = (m0 - m1) / flowRate
        
        # Oriente le vaisseau
        vessel.auto_pilot.reference_frame = node.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        vessel.auto_pilot.engage()
        vessel.auto_pilot.wait()
        
        # Accélère le temps jusqu'au burn
        burn_ut: float = ut + 60 - (burnTime/2)
        self.conn.space_center.warp_to(burn_ut)
        
        # Effectue la poussée
        vessel.control.throttle = 1.0
        time.sleep(burnTime - 0.1)
        vessel.control.throttle = 0.05
        time.sleep(0.2)
        vessel.control.throttle = 0.0
        node.remove()
        logging.info("Transfert de Hohmann effectué vers " + target_body.name)


