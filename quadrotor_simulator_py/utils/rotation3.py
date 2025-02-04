import numpy as np

from numpy import arctan2 #as atan2
from numpy import arcsin #as asin
from numpy import cos as cos
from numpy import sin as sin

from quadrotor_simulator_py.utils.quaternion import Quaternion


class Rotation3:

    def __init__(self, R=None):
        self.R = None

        if R is None:
            self.R = np.eye(3)
        else:
            self.R = R

    def to_euler_zyx(self):
        """ Convert self.R to Z-Y-X euler angles

        Output:
            zyx: 1x3 numpy array containing euler angles.
                The order of angles should be phi, theta, psi, where
                roll == phi, pitch == theta, yaw == psi
        """

        phi = np.arctan2(self.R[2, 1], self.R[2, 2])
        theta = np.arcsin(-self.R[2, 0])
        psi = np.arctan2(self.R[1, 0], self.R[0, 0])
        # TODO: Assignment 1, Problem 1.1

        zyx = np.array([phi, theta, psi])
        #print(zyx)

        return zyx

    @classmethod
    def from_euler_zyx(self, zyx):
        """ Convert euler angle rotation representation to 3x3
                rotation matrix. The input is represented as 
                np.array([roll, pitch, yaw]).
        Arg:
            zyx: 1x3 numpy array containing euler angles

        Output:
            Rot: 3x3 rotation matrix (numpy)
        """

        # TODO: Assignment 1, Problem 1.2
        #print(zyx)
        phi, theta, psi = zyx
        #print(psi)
        Rz = np.array([
            [cos(psi), -sin(psi), 0],
            [sin(psi), cos(psi), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ])

        Rx = np.array([
            [1, 0, 0],
            [0, cos(phi), -sin(phi)],
            [0, sin(phi), cos(phi)]
        ]) 

        Rot = Rotation3()
        #Rot.R = np.eye(3)
        Rot.R = Rz @ Ry @ Rx
        return Rot

    def roll(self):
        """ Extracts the phi component from the rotation matrix

        Output:
            phi: scalar value representing phi
        """

        # TODO: Assignment 1, Problem 1.3

        return np.arctan2(self.R[2, 1], self.R[2, 2])

    def pitch(self):
        """ Extracts the theta component from the rotation matrix

        Output:
            theta: scalar value representing theta
        """

        # TODO: Assignment 1, Problem 1.4

        return np.arcsin(-self.R[2, 0])

    def yaw(self):
        """ Extracts the psi component from the rotation matrix

        Output:
            theta: scalar value representing psi
        """

        # TODO: Assignment 1, Problem 1.5

        return np.arctan2(self.R[1, 0], self.R[0, 0])

    @classmethod
    def from_quat(self, q):
        """ Calculates the 3x3 rotation matrix from a quaternion
                parameterized as (w,x,y,z).

        Output:
            Rot: 3x3 rotation matrix represented as numpy matrix
        """

        # TODO: Assignment 1, Problem 1.6
        w,x,y,z = q.w(), q.x(), q.y(), q.z()
       
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])
        Rot = Rotation3(R)
        #Rot.R = np.eye(3)
        return Rot

    def to_quat(self):
        """ Calculates a quaternion from the class variable
                self.R and returns it

        Output:
            q: An instance of the Quaternion class parameterized
                as [w, x, y, z]
        """
        #reference: Craig, J. J.: "Introduction to Robotics: Mechanics and Control," Pearson.

        # TODO: Assignment 1, Problem 1.7

        R = self.R
        trace = np.trace(R)

        if trace > 0:
            S = 2.0 * np.sqrt(trace + 1.0)
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

        return Quaternion([w, x, y, z])
