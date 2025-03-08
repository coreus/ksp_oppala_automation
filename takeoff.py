
import logging, time, os, sys
from random import randint, uniform
from flightplan import FlightPlan
from connect import Connect


plan = sys.argv[1]

conn = Connect.start('Takeoff')

logging.basicConfig(level=logging.DEBUG)



success = False
maxRemainingFuel = 0
targetAltitude = 80000

turnStartAltitude = 1100
turnEndAltitude = 48000
alreadyFlying = round(conn.space_center.active_vessel.flight(conn.space_center.active_vessel.orbit.body.reference_frame).speed) != 0

conn.close()
conn = Connect.start('Takeoff')
flight = FlightPlan(conn)


#Flight sequence


if not alreadyFlying:
    time.sleep(2)
    if plan == "test":
        logging.info("test launch")
        flight.testLaunch(6000)
    if plan == "orbit":
        logging.info("launch to orbit")
        flight.launchToOrbit(targetAltitude,turnStartAltitude,turnEndAltitude)
        logging.info("desorbit")
        flight.landAtKSC()
        flight.desorbit(4)
    conn.space_center.quicksave()
    alreadyFlying = True
if plan == "orbit":
    flight.reentry("reentry_plan.csv")
    flight.setFrontFlap(30)
    flight.setBackFlap(90)
if plan == "test":
    flight.prepareLanding()
logging.info("landing")
flight.landing()
flight.touchDown()
