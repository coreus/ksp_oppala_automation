import krpc.stream
import time
class Derivative:
    def __init__(self, stream) -> None:
        self.stream = stream

    def getDelta(self) -> float:
        t0 = self.stream()
        time.sleep(0.1)
        t1 = self.stream()
        return (t1 - t0) / 0.1
    