from abstract_tuning import AbstractTuning

class FlapTuningV2(AbstractTuning):
    def __init__(self,getFrontFlapAngle,getBackFlapAngle,setFrontFlapAngle,setBackFlapAngle,getPitch) -> None:
        super().__init__(getPitch)
        self.getFrontFlapAngle = getFrontFlapAngle
        self.getBackFlapAngle = getBackFlapAngle
        self.setFrontFlapAngle = setFrontFlapAngle
        self.setBackFlapAngle = setBackFlapAngle
        
        self.tuneFront = True
        self.tuneBack = True
        self.threshold = 1
        self.reverse = True
        self.step = 0.5
        self.reset()
        
    def setFrontState(self, auto) -> None:
        self.tuneFront = auto

    def setBackState(self, auto) -> None:
        self.tuneBack = auto

    def setReverse(self,reverse) -> None:
        self.reverse = reverse

    def setStep(self,step) -> None:
        self.step = step

    def update(self,k):
        self.tuneFront and self.setFrontFlapAngle(self.getFrontFlapAngle() + k * self.step)
        if self.reverse :
            self.tuneBack and self.setBackFlapAngle(self.getBackFlapAngle() - k * self.step)
        else:
            self.tuneBack and self.setBackFlapAngle(self.getBackFlapAngle() + k *self.step)