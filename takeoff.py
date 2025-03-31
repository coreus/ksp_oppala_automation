
import logging, time, os, sys
import math
from flightplan import FlightPlan
from rendezvous import Rendezvous
from connect import Connect

plan = sys.argv[1]

conn = Connect.start('Takeoff')

logging.basicConfig(level=logging.DEBUG)


timeToOrbit = 227.8
longitudeAtOrbit=-63.79248195922855


success = False
maxRemainingFuel = 0
targetAltitude = 80000

turnStartAltitude = 1100
turnEndAltitude = 48000

vesselName = sys.argv[2]
conn.close()
conn = Connect.start('Takeoff')
try:
    vessel = conn.space_center.active_vessel
except:
    print("vessel not found")
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
        target = sys.argv[2]
        vessels = conn.space_center.vessels 
        target_vessel = next((vessel for vessel in vessels if vessel.name == target), None)
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
            #degreesDifference = (target_vessel.flight().longitude - startAtLongitude + 360) % 360
            
            #if degreesDifference > 0:
            #    degreesDifference -= 360
            
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
        conn.close()
        conn = Connect.start('Landing')
        flight = FlightPlan(conn)
        flight.landAtKSC()
        flight.desorbit(4)
        flight.reentry("reentry_plan.csv")
        flight.setFrontFlap(30)
        flight.setBackFlap(90)
        logging.info("landing")
        flight.landing()
        flight.touchDown()
        exit()
        
    if plan == "testorbit":
        logging.info("launch to orbit")
        flight.launchToOrbit(targetAltitude,turnStartAltitude,turnEndAltitude)
        logging.info("desorbit")
        
    conn.space_center.quicksave()
    alreadyFlying = True
if plan == "testorbit":
    flight.landAtKSC()
    flight.desorbit(4)
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
if plan == "rendezvous":
    target = sys.argv[2]
    vessels = conn.space_center.vessels 
    target_vessel = next((vessel for vessel in vessels if vessel.name == target), None)
    if target_vessel == None:
        logging.error("No target vessel found")
        exit()
    targetFlight = FlightPlan(conn,target_vessel)
    rdv = Rendezvous(flight, targetFlight)
    rdv.execute()
    conn.close()
    conn = Connect.start('Landing')
    flight.landAtKSC()
    flight.desorbit(4)
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
if plan == "test":
    flight.prepareLanding()
logging.info("landing")
flight.landing()
flight.touchDown()
