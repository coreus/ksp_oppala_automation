import logging, time, os, sys
import math
from libs.flightplan import FlightPlan
from libs.rendezvous import Rendezvous
from libs.connect import Connect
from libs.logger import setup as setup_logging

plan = sys.argv[1]
vesselName = sys.argv[2]
target = sys.argv[3]
conn = Connect.start('Takeoff')

setup_logging()


timeToOrbit = 227.8
longitudeAtOrbit=-63.79248195922855


success = False
maxRemainingOxidizer = 500
maxRemainingFuel = 410

targetAltitude = 80000

turnStartAltitude = 1100
turnEndAltitude = 48000


conn.close()
conn = Connect.start('Takeoff')
try:
    vessel = conn.space_center.active_vessel
except:
    print("vessel not found, summon new ship")
    conn.space_center.launch_vessel("VAB", vesselName, "LaunchPad", True, [""])
flight = FlightPlan(conn)
flight.control()

alreadyFlying = round(conn.space_center.active_vessel.flight(conn.space_center.active_vessel.orbit.body.reference_frame).speed) != 0
#Flight sequence


if not alreadyFlying:
    time.sleep(2)
    if plan == "test":
        logging.info("test launch")
        flight.testLaunch(6000)
    if plan == "orbit":
        logging.info("launch to orbit")
        flight.launchToOrbit(targetAltitude,turnStartAltitude,turnEndAltitude)
        exit()
    if plan =="rendezvous":
        refill = True
        while refill:
            vessels = conn.space_center.vessels 
            target_vessel = next((vessel for vessel in vessels if vessel.name == target and vessel != conn.space_center.active_vessel), None)
            if target_vessel == None:
                logging.error("No target vessel found")
                exit()
            altitude=target_vessel.orbit.body.equatorial_radius + target_vessel.flight().mean_altitude
            perimeter=altitude*2*math.pi
            meterPerDegree=perimeter/360
            degreesPerSecond=target_vessel.orbit.orbital_speed/meterPerDegree
            degreesDuringLaunch=timeToOrbit*degreesPerSecond
            startAtLongitude=longitudeAtOrbit-degreesDuringLaunch
            logging.info(f"wait for longitude {startAtLongitude}")
            
            while True:
                degreesDifference = (startAtLongitude  - target_vessel.flight().longitude) % 360
                logging.info(f"degrees difference: {degreesDifference}")
                conn.space_center.warp_to(conn.space_center.ut + abs(degreesDifference) / degreesPerSecond)
                degreesPerSecond=target_vessel.orbit.orbital_speed/meterPerDegree
                degreesDuringLaunch=timeToOrbit*degreesPerSecond
                startAtLongitude=longitudeAtOrbit-degreesDuringLaunch
                if abs((target_vessel.flight().longitude - startAtLongitude + 360) % 360) < 10:
                    break
            
            while target_vessel.flight().longitude < math.floor(startAtLongitude) or target_vessel.flight().longitude > math.ceil(startAtLongitude):
                logging.info(f"longitude {target_vessel.flight().longitude}")
                time.sleep(1)
            
            time.sleep(20)
            flight.launchToOrbit(targetAltitude,turnStartAltitude,turnEndAltitude)
            
            targetFlight = FlightPlan(conn,target_vessel)
            rdv = Rendezvous(flight, targetFlight)
            rdv.execute()
            time.sleep(10)
            rdv.transfer(maxRemainingOxidizer,maxRemainingFuel)
            time.sleep(1)
            rdv.undock()
            
            conn.close()
            conn = Connect.start('Landing')
            if conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount != maxRemainingFuel:
                tank_vessel = next((vessel for vessel in conn.space_center.vessels if vessel.name == vesselName and vessel != conn.space_center.active_vessel), None)
                conn.space_center.active_vessel = tank_vessel

            target_vessel = next((vessel for vessel in conn.space_center.vessels if vessel.name == target), None)
            refill =  (target_vessel.resources.with_resource("LiquidFuel")[0].amount / target_vessel.resources.with_resource("LiquidFuel")[0].max < 0.9)
                

            flight = FlightPlan(conn)
            flight.landAtKSCSequence()
            flight.desorbit(4)
            flight.reentry("reentry_plan.csv")
            flight.setFrontFlap(30)
            flight.setBackFlap(90)
            logging.info("landing")
            flight.landing(700)
            flight.touchDown()
            flight.vessel.recover()
            if refill:
                conn.close()
                conn = Connect.start('Takeoff')
                conn.space_center.launch_vessel("VAB", vesselName, "LaunchPad", True, [""])
                flight = FlightPlan(conn)
                flight.control()
            
        exit()
        
    if plan == "testorbit":
        logging.info("launch to orbit")
        flight.launchToOrbit(targetAltitude,turnStartAltitude,turnEndAltitude)
        logging.info("desorbit")
        
    conn.space_center.quicksave()
    alreadyFlying = True
if plan == "transfer":
    flight.deploySolarPanels()
    flight.openDockingPort(False)
    flight.deployAntenna()
    origin_body_name = conn.space_center.active_vessel.orbit.body.name
    
    #conn.space_center.load_space_center()
    
    flight.waitForHohmannTransfer(conn.space_center.bodies[origin_body_name], conn.space_center.bodies[target])

    #flight.executeHohmannTransfer(conn.space_center.bodies[target])
    exit()
if plan == "testorbit":
    flight.landAtKSCSequence()
    flight.desorbit(4)
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
    flight.landing(700)
    flight.touchDown()
    exit()
if plan == "rendezvous":
    vessels = [vessel for vessel in conn.space_center.vessels if vessel.name == target]
    target_vessel = next((vessel for vessel in vessels if vessel.name == target and vessel != conn.space_center.active_vessel), None)
    
    if target_vessel == None:
        logging.error("No target vessel found")
        exit()
    targetFlight = FlightPlan(conn,target_vessel)
    rdv = Rendezvous(flight, targetFlight)
    rdv.execute()
    
    conn.close()
    conn = Connect.start('Landing')
    if conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount != maxRemainingFuel:
        tank_vessel = next((vessel for vessel in conn.space_center.vessels if vessel.name == target and vessel != conn.space_center.active_vessel), None)
        conn.space_center.active_vessel = tank_vessel
    flight = FlightPlan(conn)
    flight.landAtKSCSequence()
    flight.desorbit(4)
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
if plan == "test":
    flight.prepareLanding()
if plan == "reentry":
    flight.landAtKSCSequence()
    flight.desorbit(4)
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
logging.info("landing")
flight.landing(700)
flight.touchDown()
