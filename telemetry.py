import krpc
import krpc.stream

class Telemetry:
    def __init__(self,conn: krpc.Client) -> None:
        self.conn = conn
        self.vessel = self.conn.space_center.active_vessel

    def streamAltitude(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')

    def streamFuel(self,pstage:int,pcumulaTive=False) -> krpc.stream:
        return self.conn.add_stream(self.vessel.resources_in_decouple_stage(stage=pstage,cumulative=pcumulaTive).amount,'LiquidFuel')
    
    def streamApoapsis(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')

    def streamTimeToApoapsis(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.orbit, 'time_to_apoapsis')
    
    def streamGForce(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(), 'g_force')
    
    def streamVelocity(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'velocity')
    
    def streamSpeed(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'speed')
    
    def streamHorizontalSpeed(self,reference_frame) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(reference_frame), 'horizontal_speed')
    
    def streamVerticalSpeed(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'vertical_speed')
    
    def streamLatitude(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'latitude')
    
    def streamLongitude(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(self.vessel.orbit.body.reference_frame), 'longitude')
    
    def streamUT(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.conn.space_center, 'ut')
    
    def streamPitch(self) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(), 'pitch')
    
    def streamVelocity(self, reference_frame) -> krpc.stream:
        return self.conn.add_stream(getattr, self.vessel.flight(reference_frame), 'velocity')