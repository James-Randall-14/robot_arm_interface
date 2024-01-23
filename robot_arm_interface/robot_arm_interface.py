#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from adafruit_servokit import ServoKit
import RPi.GPIO
import time

class Arm_Interface(Node):

    def __init__(self):
        super().__init__("robot_arm_interface")

        self.log = self.get_logger()

        RPi.GPIO.setwarnings(False)
        
        self.kit = ServoKit(channels=16)

        self.create_servo_params()
        self.servo_command_values = [90] * 6

        self.tolerance = 1.0

        self.timer = rclpy.timer.Timer(self.command_servos(), 0.005)

        # Declare parameters callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.command_servos()

    def create_servo_params(self):
        parameter_bounds = FloatingPointRange()
        parameter_bounds.from_value = 0.0
        parameter_bounds.to_value = 180.0
        parameter_bounds.step = 0.1
        parameter_descriptor = ParameterDescriptor(floating_point_range = [parameter_bounds])

        for i in range(6):
            i += 1
            self.declare_parameter('servo_{}'.format(i), 90.0, parameter_descriptor)
    
    def parameters_callback(self, params):
        
        for param in params:
            if param.name == "use_sim_time": pass
            else:

                # Grab the servo index from the name of the parameter
                index = int(param.name.replace('servo_', '')) - 1

                self.servo_command_values[index] = param.value
        
        return SetParametersResult(successful=True)

    # Commands servos to move at a much slower speed than max
    # This means the arm is not jerking around all of the time
    def command_servos(self):

        servo_states = self.get_servo_states()
        
        if self.is_good_enough(servo_states, self.servo_command_values):
            return
        
        # Increment each servo position
        for i in range(len(self.servo_command_values)):

            command_val = self.servo_command_values[i]
            servo_state = servo_states[i]

            # If the servo position is outside of the recognized ranges, just assume angle
            if servo_state is None:
                self.kit.servo[i].angle = command_val

            # Only adjust position if the servo is not already at target
            if not self.is_good_enough([servo_state], [command_val]):
                difference = command_val - servo_state
                self.log.info("Difference: {}".format(difference))
            else: difference = 0

            # Temporary slow mechanism till we get PID control
            increment = difference / 12
            
            # Make sure that increment never passes below the inaction threshold
            if abs(increment) < 0.6:
                if increment < 0: increment = -0.6
                else: increment = 0.6 
            # If it's close enough to the edge that it doesn't matter
            if self.kit.servo[i].angle + increment > 180 or self.kit.servo[i].angle + increment < 0:
                increment = 0

            # Adjust servo position
            self.kit.servo[i].angle += increment

    # Compares the servo states and their command values. If it's within tolerance, return true
    def is_good_enough(self, states, commands):
        for i in range(len(states)):
            if abs(states[i] - commands[i]) > self.tolerance:
                return False
        return True

    # Get servo states into a list
    def get_servo_states(self):
        servo_states = []
        for servo in self.kit.servo:
            servo_states.append(servo.angle)
        return servo_states
        
# Run the node
def main(args=None):
    rclpy.init(args=args)

    interface = Arm_Interface()

    rclpy.spin(interface)
    
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()