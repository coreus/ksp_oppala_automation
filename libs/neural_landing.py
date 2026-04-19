import tensorflow as tf
import numpy as np



class NeuralLanding:
    def __init__(self,path:str) -> None:
        self.modelPath = path
        self.model = tf.keras.models.load_model(self.modelPath)

    #def getAngle(self,backFlapAngle:float,pitch:float,deltaPitch:float,atmoDensity:float):
    def getAngle(self,pitch:float,deltaPitch:float,atmoDensity:float,horizontalSpeed:float,verticalSpeed:float):
        #backFlapAngle = backFlapAngle / 90
        pitch = pitch / 180
        deltaPitch = deltaPitch / 90
        horizontalSpeed = horizontalSpeed / 150
        verticalSpeed = verticalSpeed / 150
        #prediction = self.model.predict(np.array([backFlapAngle,pitch,deltaPitch,atmoDensity]))[0]
        prediction = self.model.predict(np.array([pitch,deltaPitch,atmoDensity,horizontalSpeed,verticalSpeed]))[0]
        back = prediction[0] * 90
        front = prediction[1] * 90
        return [back,front]
        #return self.model.predict(np.array([backFlapAngle,pitch,deltaPitch,atmoDensity]))[0][0] * 90

    
