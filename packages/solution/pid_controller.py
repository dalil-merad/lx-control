#!/usr/bin/env python3

from typing import Tuple
import numpy as np

class PIDController():
    def __init__(self):

        # We will initialize some variables that might be useful
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = 0.0
        self.prev_int_offset = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0


    def HeadingControl(self,
                       v_ref: float,
                       theta_ref: float,
                       theta_curr: float,
                       delta_t: float
    ) -> Tuple[float, float]:
        """
        PID performing heading control.
        Args:
            v_ref:      reference velocity.
            theta_ref:  reference heading pose.
            theta_curr: the current estimated heading.
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference heading
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_heading to track the integral term
        # self.prev_e_heading the previous error. But note that you
        # should be the one to update them also.
    
        v = v_ref

        theta_err = theta_ref - theta_curr
        print(f'Current theta = {theta_curr}')
        print(f'error = {theta_err}')

        #Proportional
        prop = self.kp * theta_err

        #Integral
        new_int_error_heading = self.prev_int_heading + theta_err * delta_t
        print(f'Integral Error: {new_int_error_heading}\n')
        self.prev_int_heading = new_int_error_heading
        integral = self.ki * (new_int_error_heading)

        #Derivative
        de_dt = (theta_err - self.prev_e_heading)/ delta_t
        deriv = self.kd* de_dt
        
        self.prev_e_heading = theta_err

        omega = prop + integral + deriv

        print(f'Kp:{self.kp}\n Ki:{self.ki}\nKd:{self.kd}')

        return v, omega

    def OffsetControl(self,
                      v_ref: float,
                      y_ref: float,
                      y_curr: float,
                      delta_t: float,
                      ) -> Tuple[float, float]:
        """
        PID performing lateral offset control.
        Args:
            v_ref:      linear Duckiebot speed.
            y_ref:      reference heading pose.
            y_curr:     the current estimated "y" coordinate (offset)
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference lateral offset
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_offset to track the integral term
        # self.prev_e_offset the previous error. But note that you
        # should be the one to update them also.

        v = v_ref

        y_err = y_ref - y_curr
        #print(f'Current y= {y_curr}')
        #print(f'error = {y_err}')

        #Proportional
        prop = self.kp * y_err

        #Integral
        new_int_error_offset= self.prev_int_offset + y_err * delta_t
        #print(f'Integral Error: {new_int_error_offset}')
        self.prev_int_offset = new_int_error_offset
        integral = self.ki * (new_int_error_offset)

        #Derivative
        de_dt = (y_err - self.prev_e_offset)/ delta_t
        #print(f'Derivative Error: {de_dt}')
        deriv = self.kd* de_dt
        
        self.prev_e_offset = y_err

        omega = prop + integral + deriv

        #print(f'Kp:{self.kp}\n Ki:{self.ki}\nKd:{self.kd}\n')


        return v, omega

    def SetGains(self, kp: float = None, ki: float = None, kd: float = None, reset_integral=False) -> None:
        # Set the PID gains
        if kp is not None and ki is not None and kd is not None and not reset_integral:
            self.kp = kp
            self.ki = ki
            self.kd = kd
        elif reset_integral:
            self.prev_int_heading = 0.0
            self.prev_int_offset = 0.0
            self.prev_e_heading = 0.0
            self.prev_e_offset = 0.0