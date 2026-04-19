import numpy as np
import math

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
    
    def v3minus(v,t):
        '''
        vector subtraction
        '''
        a = v[0]-t[0]
        b = v[1]-t[1]
        c = v[2]-t[2]
        return (a,b,c)

    def speed(v,t):
        '''
        returns speed (magnitude) between two
        velocities
        '''
        rf = v.orbit.body.non_rotating_reference_frame
        vec = Helper.v3minus(v.velocity(rf), t.velocity(rf))
        a = vec[0] * vec[0]
        b = vec[1] * vec[1]
        c = vec[2] * vec[2]
        return math.sqrt(a+b+c)

    def dist(v,t):
        '''
        returns distance (magnitude) between two
        positions
        '''
        rf = v.orbit.body.non_rotating_reference_frame
        vec = Helper.v3minus(v.position(rf), t.position(rf))
        a = vec[0] * vec[0]
        b = vec[1] * vec[1]
        c = vec[2] * vec[2]
        return math.sqrt(a+b+c)
