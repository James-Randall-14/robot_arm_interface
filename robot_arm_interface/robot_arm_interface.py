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

        self.command_servos()
        
        return SetParametersResult(successful=True)

    # Commands servos to move at a much slower speed than max
    # This means the arm is not jerking around all of the time
    # Could go more complex with like a PID controller but not worth
    # Because motor state feedback is nonexistent
    def command_servos(self):
        for i in range(len(self.servo_command_values)):
            command_val = self.servo_command_values[i]
            good_enough = False
            while (self.kit.servo[i].angle != command_val) and good_enough == False:

                # Handle when servo has been adjusted to value outside of range manually
                if self.kit.servo[i].angle is None: 
                    self.kit.servo[i].angle = command_val
                    differnece = 0
                else: difference = command_val - self.kit.servo[i].angle

                
                if abs(difference) < 1:
                    good_enough = True
                else: pass

                self.log.info("Difference: {}".format(difference))
                increment = difference / 12
                # Make sure increment never passes below the inaction threshold
                # Maintains the current sign
                if abs(increment) < 0.6:
                    if increment < 0: increment = -0.6
                    else: increment = 0.6
                if self.kit.servo[i].angle + increment > 180 or self.kit.servo[i].angle + increment < 0:
                    good_enough = True
                else:
                    self.kit.servo[i].angle += increment
                time.sleep(0.005)

# Run the node
def main(args=None):
    rclpy.init(args=args)

    interface = Arm_Interface()

    rclpy.spin(interface)
    
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()