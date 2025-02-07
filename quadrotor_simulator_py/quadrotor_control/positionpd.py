#!/usr/bin/env python
import numpy as np
from numpy import sin, cos
from numpy.linalg import norm
import yaml

from quadrotor_simulator_py.quadrotor_control.state import State
from quadrotor_simulator_py.quadrotor_control.trackingerror import TrackingError
from quadrotor_simulator_py.quadrotor_model.mixer import QuadMixer
from quadrotor_simulator_py.quadrotor_control.cascaded_command import CascadedCommand
from quadrotor_simulator_py.utils import Quaternion
from quadrotor_simulator_py.utils import shortest_angular_distance


class QuadrotorPositionControllerPD:

    def __init__(self, yaml_file):
        self.zw = np.array([[0], [0], [1]])  # unit vector [0, 0, 1]^{\top}
        self.gravity_norm = 9.81
        self.current_state = State()
        self.state_ref = State()
        self.tracking_error = TrackingError()

        self.Rdes = np.eye(3)
        self.Rcurr = None
        self.accel_des = 0.0
        self.angvel_des = np.zeros((3, 1))
        self.angacc_des = np.zeros((3, 1))
        self.mass = 0.0

        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

        self.mass = data['mass']

        self._Kx = np.eye(3)
        self._Kx[0, 0] = data['gains']['pos']['x']
        self._Kx[1, 1] = data['gains']['pos']['y']
        self._Kx[2, 2] = data['gains']['pos']['z']

        self._Kv = np.eye(3)
        self._Kv[0, 0] = data['gains']['vel']['x']
        self._Kv[1, 1] = data['gains']['vel']['y']
        self._Kv[2, 2] = data['gains']['vel']['z']

        self.gravity_norm = data['gravity_norm']

    def update_state(self):
        self.Rcurr = self.current_state.rot

    def set_current_state(self, state_in):
        self.current_state = state_in
        self.update_state()

    def get_state(self):
        return self.current_state

    def set_reference_state(self, ref_in):
        self.state_ref = ref_in

    def compute_body_z_accel(self, a_des, R_curr):
        """ Calculates the body-frame z-acceleration

        Args:
            a_des: 3x1 numpy array representing the desired acceleration
            R_curr: 3x3 rotation matrix representing Rwb

        Output:
            u: scalar value representing body-frame z-acceleration
        """

        # TODO: Assignment 1, Problem 2.1
        body_z = R_curr[:, 2]  # Extract third column (z-axis in world frame)
        u = np.dot(body_z, a_des)

        return u

    def compute_orientation(self, a_des, yaw_ref):
        """ Calculates the desired orientation

        Args:
            a_des: 3x1 numpy array representing the desired acceleration
            yaw_ref: yaw reference

        Output:
            R_des: 3x3 numpy matrix representing desired orientation
        """

        # TODO: Assignment 1, Problem 2.2
        # zb_des = a_des / norm(a_des)  
        # xc_des = np.array([cos(yaw_ref), sin(yaw_ref), 0])

        # yb_des = np.cross(zb_des.flatten(), xc_des)
        # yb_des /= norm(yb_des)  # Normalize

        # xb_des = np.cross(yb_des, zb_des.flatten())
        # xb_des /= norm(xb_des)  # Normalize

        # yb_des = np.cross(zb_des.flatten(), xb_des)

        # R_des = np.column_stack((xb_des, yb_des, zb_des.flatten()))

        y_c = np.array([-np.sin(yaw_ref), np.cos(yaw_ref), 0]).reshape(-1, 1)
        z_b_des = a_des / np.linalg.norm(a_des)
        
        x_b_des = np.cross(y_c.flatten(), z_b_des.flatten()).reshape(-1, 1)
        x_b_des /= np.linalg.norm(x_b_des)

        y_b_des = np.cross(z_b_des.flatten(), x_b_des.flatten()).reshape(-1, 1)
        R_des = np.column_stack((x_b_des, y_b_des, z_b_des))

        #R_des = np.eye(3)
        return R_des

    def compute_hod_refs(self, acc_vec_des, flat_ref, R_des):
        """ Calculates the desired angular velocities and accelerations.

        Args:
            acc_vec_des: 3x1 numpy array representing the desired acceleration
            flat_ref: class instance of State() containing the trajectory reference
            R_des: desired rotation

        Output:
            angvel_des: 3x1 numpy array representing desired angular velocity
            angacc_des: 3x1 numpy array representing desired angular acceleration
        """

        # TODO: Assignment 1, Problem 2.3

        # angvel_des = np.zeros((3, 1))
        # angacc_des = np.zeros((3, 1))
        # jerk_des = flat_ref.jerk
        # snap_des = flat_ref.snap
        
        # angvel_des = np.dot(R_des.T, jerk_des) / norm(acc_vec_des)
        # angacc_des = np.dot(R_des.T, snap_des) / norm(acc_vec_des)

        z_des = R_des[:, 2].reshape(-1, 1)
        y_des = R_des[:, 1].reshape(-1, 1)
        x_des = R_des[:, 0].reshape(-1, 1)
        
        c = np.dot(z_des.T, acc_vec_des)[0, 0]

        w_x = -np.dot(y_des.T, flat_ref.jerk)[0, 0] / c
        w_y = np.dot(x_des.T, flat_ref.jerk)[0, 0] / c

        x_c = np.array([np.cos(flat_ref.yaw), np.sin(flat_ref.yaw), 0]).reshape(-1, 1)
        y_c = np.array([-np.sin(flat_ref.yaw), np.cos(flat_ref.yaw), 0]).reshape(-1, 1)

        w_z_numerator = (flat_ref.dyaw * np.dot(x_c.T, x_des)[0, 0] + w_y * np.dot(y_c.T, z_des)[0, 0])
        w_z_denominator = np.linalg.norm(np.cross(y_c.flatten(), z_des.flatten()).reshape(-1, 1))
        w_z = w_z_numerator / w_z_denominator

        angvel_des = np.array([[w_x], [w_y], [w_z]])

        c_dot = np.dot(z_des.T, flat_ref.jerk)[0, 0]

        a_x = (-np.dot(y_des.T, flat_ref.snap)[0, 0] - (2 * c_dot * w_x) + (c * w_y * w_z)) / c
        a_y = (np.dot(x_des.T, flat_ref.snap)[0, 0] - (2 * c_dot * w_y) - (c * w_x * w_z)) / c

        a_z_numerator = (flat_ref.d2yaw * np.dot(x_c.T, x_des)[0, 0]
                        - (2 * flat_ref.dyaw * w_y * np.dot(x_c.T, z_des)[0, 0])
                        + (2 * flat_ref.dyaw * w_z * np.dot(x_c.T, y_des)[0, 0])
                        - (w_x * w_y * np.dot(y_c.T, y_des)[0, 0])
                        - (w_x * w_z * np.dot(y_c.T, z_des)[0, 0])
                        + (a_y * np.dot(y_c.T, z_des)[0, 0]))
        a_z_denominator = np.linalg.norm(np.cross(y_c.flatten(), z_des.flatten()).reshape(-1, 1))
        a_z = a_z_numerator / a_z_denominator

        angacc_des = np.array([[a_x], [a_y], [a_z]])

        return angvel_des, angacc_des

    def compute_command(self):
        """ This function contains the following functionality:
                1. Computes the PD feedback-control terms from the position
                   and velocity control errors.
                2. Computes the desired rotation using compute_orientation.
                3. Applies the thrust command to the body frame using
                   compute_body_z_accel
                4. Calculates the desired angular velocities and accelerations.
        """

        # TODO: Assignment 1, Problem 2.4
        pos_err = self.current_state.pos - self.state_ref.pos
        vel_err = self.current_state.vel - self.state_ref.vel
        
        accel_des = -np.dot(self._Kx, pos_err) - np.dot(self._Kv, vel_err) + self.state_ref.acc + self.gravity_norm * self.zw
        self.accel_des = self.compute_body_z_accel(accel_des, self.Rcurr)
        
        self.Rdes = self.compute_orientation(accel_des, self.state_ref.yaw)
        self.angvel_des, self.angacc_des = self.compute_hod_refs(accel_des, self.state_ref, self.Rdes)
        pass

    def get_cascaded_command(self):
        casc_cmd = CascadedCommand()
        casc_cmd.thrust_des = self.mass * self.accel_des
        casc_cmd.Rdes = self.Rdes
        casc_cmd.angvel_des = self.angvel_des
        casc_cmd.angacc_des = self.angacc_des
        return casc_cmd

    def get_tracking_error(self):
        return self.tracking_error

    def update_tracking_error(self):
        self.tracking_error = TrackingError()
        self.tracking_error.pos_des = self.state_ref.pos
        self.tracking_error.vel_des = self.state_ref.vel
        self.tracking_error.acc_des = self.state_ref.acc
        self.tracking_error.jerk_des = self.state_ref.jerk
        self.tracking_error.snap_des = self.state_ref.snap
        self.tracking_error.yaw_des = self.state_ref.yaw
        self.tracking_error.dyaw_des = self.state_ref.dyaw
        self.tracking_error.pos_err = self.current_state.pos - self.state_ref.pos
        self.tracking_error.vel_err = self.current_state.vel - self.state_ref.vel
        self.tracking_error.yaw_err = shortest_angular_distance(
            self.current_state.yaw, self.state_ref.yaw)
        self.tracking_error.dyaw_err = self.current_state.dyaw - self.state_ref.dyaw

    def run_ctrl(self):

        # get updated state
        self.update_state()

        # calculate the command
        self.compute_command()

        # update tracking error
        self.update_tracking_error()
