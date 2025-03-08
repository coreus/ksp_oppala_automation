import numpy as np

class Helper:
    
    def normalizeVector(vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)


    def calculateAngle(v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1 = Helper.normalizeVector(np.array(v1))
        v2 = Helper.normalizeVector(np.array(v2))
        return np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))