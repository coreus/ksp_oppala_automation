import krpc.stream
import time
class Derivative:
    def __init__(self, value, deltaT = 0.1) -> None:
        self.isStream = False
        if isinstance(value, krpc.stream.Stream) or callable(value):
            self.isStream = True
        self.value = value
        self.deltaT = deltaT

    def getDelta(self, param=None) -> float:
        if self.isStream:
            t0 = self.value()
        else:
            t0 = self.value
            
        time.sleep(0.1)
        if self.isStream:
            t1 = self.value()
        else:
            t1 = self.value
        if param:
            return (t1[param] - t0[param]) / self.deltaT
        return (t1 - t0) / self.deltaT
    