from typing import List, Tuple
import math
import numpy as np


class WallFollower:
    """Class to safely explore an environment (without crashing) when the pose is unknown."""

    def __init__(self, dt: float):
        """Wall following class initializer.

        Args:
            dt: Sampling period [s].

        """
        self._dt: float = dt
        # define structure to host variables for the control strategy
        self._control = type(
            "control",
            (object,),
            {
                "kp" : 10,
                "kd" : 5,
                "ki" : 0.1
            },
        )()
        self._limits = type(
            "limits",
            (object,),
            {
                "frontal" : 0.4,
                "side" : 0.2,
                "v" : 0.2,
                "w" : 0.5
            },
        )()
        self.control_case: string = "follow_left"
        self.error_ant: float = 0.0
        self.prev_w: float = 0.0

        self

    def pid_control(self, error: float) -> float:
        """PD control.
        Args:
            error: Error signal. Difference between measurement values
        Returns:
            w: Control signal.
        """
        # PD CONTROL
        #w = kp * error + kd * (error - self.error_ant) / self._dt
        # PID CONTROL
        w = self._control.kp * error + self._control.kd * (error - self.error_ant) / self._dt + self._control.ki * error * self._dt
        self.error_ant = error


        return w

    def normalise_sensors(self, z_us: List[float]) -> List[float] :

        norm = [min(value, 1.0) for value in z_us]

        return norm

    def compute_commands(self, z_us: List[float], z_v: float, z_w: float) -> Tuple[float, float]:
        """Wall following exploration algorithm. COmprised of a state machine that checks the
        distances between the sensors and walls and changes mode accordingly.
        z_us[3], z_us[4] : front sensors
        z_us[0], z_us[15] : left side sensors 
        z_us[7], z_us[8] : right side sensors
        z_us[11], z_us[12] : back sensors       
        

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            z_v: Odometric estimate of the linear velocity of the robot center [m/s].
            z_w: Odometric estimate of the angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        v = 0.0
        w = 0.0
        # TODO 1.14: Implement the wall following algorithm with PD control for the angular velocity,
        # and saturation for the linear velocity.

        wall_upfront = (z_us[3]+z_us[4])/2 < self._limits.frontal
        empty_upfront = math.isinf(z_us[3]) and math.isinf(z_us[4])
        empty_left = math.isinf(z_us[0]) and math.isinf(z_us[15])
        wall_left = z_us[0] < 1 or z_us[15] < 1
        empty_right = math.isinf(z_us[7]) and math.isinf(z_us[8])
        wall_right = z_us[7] < 1 or z_us[8] < 1

        z_us = self.normalise_sensors(z_us)

        # state machine
        
        if self.control_case == "corridor" :
            # corridor navigation
            if empty_left:
                self.control_case = "follow_right"
            if empty_right:
                self.control_case = "follow_left"
            if wall_upfront and wall_left and wall_right:
                self.control_case = "turn_around"
        elif self.control_case == "follow_left" :
            # follow left wall in case any other wall dissappears
            if empty_right and wall_upfront :
                self.control_case = "turn_right"
            if empty_left:
                self.control_case = "turn left"
            else :
                self.control_case = "corridor"
        elif self.control_case == "follow_right" :
            # follow right wall in case any other wall dissappears
            if empty_left and wall_upfront :
                self.control_case = "turn_left"
            if empty_right:
                self.control_case = "turn right"
            else :
                self.control_case = "corridor"
        elif self.control_case == "turn_left" :
            # turn around left corner
            if empty_upfront and wall_left :
                self.control_case = "follow_left"
        elif self.control_case == "turn_right":
            if empty_upfront and wall_right :
                self.control_case = "follow_right"
        elif self.control_case == "turn_around":
            # turn around 
            if empty_upfront :
                self.control_case = "corridor"

        

        # control case
        if self.control_case == "corridor" :
            # corridor navigation
            w = self.pid_control(z_us[0]-z_us[7])
            v = self._limits.v
        elif self.control_case == "follow_left" :
            # follow left wall in case any other wall dissappears
            w = self.pid_control(z_us[0]-z_us[15])
            v = self._limits.v
        elif self.control_case == "follow_right" :
            # follow right wall in case any other wall dissappears
            w = self.pid_control(z_us[7]-z_us[8])
            v = self._limits.v
        elif self.control_case == "turn_left" :
            # turn around left corner
            w = self._limits.w
            v = 0
        elif self.control_case == "turn_right":
            w = -self._limits.w
            v = 0
        elif self.control_case == "turn_around":
            # turn around 
            w = self._limits.w 
            v = 0.0
            
        
        # display speeds
        print("mode: ", self.control_case)

        return float(v), float(w)
