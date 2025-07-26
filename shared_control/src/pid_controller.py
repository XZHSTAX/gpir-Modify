import time

class PIDController:
    """A class to implement a PID controller."""

    def __init__(self, kp, ki, kd):
        """Initializes the PID controller.

        Args:
            kp (float): The proportional gain.
            ki (float): The integral gain.
            kd (float): The derivative gain.
        """
        self.kp_ = kp
        self.ki_ = ki
        self.kd_ = kd

        self.previous_error_ = 0.0
        self.integral_ = 0.0
        self.use_integral = self.ki_ > 1e-5

        self.max_integral_ = float('inf')
        self.min_integral_ = float('-inf')
        self.max_output_ = float('inf')
        self.min_output_ = float('-inf')

    def set_integral_limit(self, max_val, min_val):
        """Sets the limits for the integral term.

        Args:
            max_val (float): The maximum value of the integral term.
            min_val (float): The minimum value of the integral term.
        """
        self.max_integral_ = max_val
        self.min_integral_ = min_val

    def set_output_limit(self, max_val, min_val):
        """Sets the limits for the controller's output.

        Args:
            max_val (float): The maximum output value.
            min_val (float): The minimum output value.
        """
        self.max_output_ = max_val
        self.min_output_ = min_val

    def control(self, error, dt):
        """Calculates the control output.

        Args:
            error (float): The current error.
            dt (float): The time step.

        Returns:
            float: The control output.
        """
        diff = (error - self.previous_error_) / dt

        if self.use_integral:
            self.integral_ += error * dt * self.ki_
            self.integral_ = max(min(self.integral_, self.max_integral_), self.min_integral_)

        self.previous_error_ = error
        output = error * self.kp_ + self.integral_ + diff * self.kd_
        return max(min(output, self.max_output_), self.min_output_)

    def reset(self):
        """Resets the controller's state."""
        self.integral_ = 0.0
        self.previous_error_ = 0.0