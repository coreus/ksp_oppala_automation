import logging
import time

from ksp.derivative import Derivative
from ksp.flight_plan import FlightPlan
from ksp.helper import Helper


class Rendezvous:
    def __init__(self, flight: FlightPlan, target: FlightPlan) -> None:
        self.flight = flight   # tanker / active vessel
        self.target = target

    # -------------------------------------------------------------------------
    # High-level sequences
    # -------------------------------------------------------------------------

    def execute(self) -> None:
        """Approach and dock with the target vessel."""
        logging.info("Executing rendezvous")
        self.flight.vessel.control.rcs = True
        logging.info("Closest approach distance: %.1f m", self.distanceForecast())

        while not (self.distanceForecast() < 50 and self.speed() < 1):
            while self.speed() > 1:
                self.flight.conn.space_center.warp_to(
                    self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(
                        self.target.vessel.orbit
                    )
                )
                self.nullifyRelativeSpeed()
                if (
                    self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(
                        self.target.vessel.orbit
                    ) - self.flight.ut() < 10
                ):
                    break

            maxSpeed = 6 if self.distanceForecast() > 150 else 1
            self.follow(maxSpeed)

        time.sleep(2)
        self.flight.deploySolarPanels()
        self.dock()

    def transfer(self, leftOxidizer: float = 500, leftFuel: float = 410) -> None:
        """Transfer propellant from the tanker to the target vessel."""
        self.identifyTanks()
        logging.info("Transferring fuel")
        self.flight.conn.space_center.ResourceTransfer.start(
            self.TankerLox.part, self.TargetLox.part,
            "Oxidizer", self.TankerLox.amount - leftOxidizer,
        )
        self.flight.conn.space_center.ResourceTransfer.start(
            self.TankerPropelant.part, self.TargetPropelant.part,
            "LiquidFuel", self.TankerPropelant.amount - leftFuel,
        )

    def undock(self) -> None:
        for module in self.flight.conn.space_center.active_vessel.parts.modules_with_name("ModuleDockingNode"):
            if module.has_event('Désamarrer'):
                module.trigger_event('Désamarrer')
                break
        self.flight.conn.space_center.clear_target()

    # -------------------------------------------------------------------------
    # Maneuver helpers
    # -------------------------------------------------------------------------

    def facing(self) -> None:
        logging.info("Orienting target toward tanker")
        self.target.control()
        self.target.vessel.control.rcs = True
        self.target.conn.space_center.target_vessel = self.flight.vessel
        self.target.conn.space_center.target_docking_port = self.flight.vessel.parts.docking_ports[0]
        self.target.setSAS(mode=self.target.vessel.control.sas_mode.target)
        time.sleep(5)
        self.target.activateRCS(False)

        logging.info("Orienting tanker toward target")
        self.flight.control()
        self.flight.vessel.control.rcs = True
        self.flight.conn.space_center.target_vessel = self.target.vessel
        self.flight.conn.space_center.target_docking_port = self.target.vessel.parts.docking_ports[0]
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.target)
        time.sleep(5)

    def dock(self) -> None:
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
                    if self.distance() < 5:
                        self.flight.vessel.control.forward = -1
                        while self.speed() > 0.2:
                            time.sleep(0.1)
                        self.facing()
                        self.flight.control()
                        self.flight.vessel.control.forward = 0
                        break
                    time.sleep(0.1)
            except Exception:
                continue

    def nullifyRelativeSpeed(self) -> None:
        self.flight.conn.space_center.target_vessel = self.target.vessel
        closest_approach = self.flight.vessel.orbit.distance_at_closest_approach(self.target.vessel.orbit)
        warp_target = self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(
            self.target.vessel.orbit
        )
        if closest_approach < 30:
            warp_target -= 15
        self.flight.conn.space_center.warp_to(warp_target)
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.retrograde)
        time.sleep(5)
        dspeed = Derivative(self.speed, 0.01)
        self.flight.vessel.control.throttle = 0.1
        while dspeed.getDelta() < 0:
            time.sleep(0.01)
            self.flight.vessel.control.throttle = 0.01 if self.speed() < 3 else 0.1
        self.flight.vessel.control.throttle = 0

    def follow(self, maxSpeed: float = 6) -> None:
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.target)
        time.sleep(5)
        self.flight.vessel.control.throttle = 0.1
        while self.speed() < maxSpeed:
            time.sleep(0.1)
        self.flight.vessel.control.throttle = 0
        self.flight.setSAS(mode=self.flight.vessel.control.sas_mode.retrograde)
        warp_target = self.flight.conn.space_center.active_vessel.orbit.time_of_closest_approach(
            self.target.vessel.orbit
        )
        if self.distanceForecast() < 10:
            warp_target -= 10
        self.flight.conn.space_center.warp_to(warp_target)
        self.nullifyRelativeSpeed()

    # -------------------------------------------------------------------------
    # State helpers
    # -------------------------------------------------------------------------

    def identifyTanks(self) -> None:
        count = 0
        base_lox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[0].amount
        if base_lox != self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[0].amount:
            count = 1
        self.TankerLox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[(0 + count) % 2]
        self.TargetLox = self.flight.conn.space_center.active_vessel.resources.with_resource("Oxidizer")[(1 + count) % 2]

        count = 0
        base_fuel = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount
        if base_fuel != self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[0].amount:
            count = 1
        self.TankerPropelant = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[(0 + count) % 2]
        self.TargetPropelant = self.flight.conn.space_center.active_vessel.resources.with_resource("LiquidFuel")[(1 + count) % 2]

    def speed(self) -> float:
        return Helper.speed(self.flight.vessel, self.target.vessel)

    def distance(self) -> float:
        return Helper.dist(self.flight.vessel, self.target.vessel)

    def distanceForecast(self) -> float:
        return self.flight.vessel.orbit.distance_at_closest_approach(self.target.vessel.orbit)
