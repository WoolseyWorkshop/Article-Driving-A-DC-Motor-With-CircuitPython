"""Controls two small DC motors.

Description
-----------

A CircuitPython program that interfaces with an Adafruit DRV8833 DC/Stepper
Motor Driver Breakout Board to drive two small DC motors.

Circuit
-------

- An Adafruit DRV8833 DC/Stepper Motor Driver Breakout Board is connected as
  shown below.
    - The SLP pin is connected to pin D5.
    - The FLT pin is connected to pin D6.
    - The AIN1 pin is connected to pin D9.
    - The AIN2 pin is connected to pin D10.
    - The BIN1 pin is connected to pin D11.
    - The BIN2 pin is connected to pin D12.
    - The AOUT1 pin is connected to motor A's positive pin.
    - The AOUT2 pin is connected to motor A's negative pin.
    - The BOUT1 pin is connected to motor B's positive pin.
    - The BOUT2 pin is connected to motor B's negative pin.
    - The VMOTOR+ pin is connected to a 6-9 volt power supply.
- A 10K potentiometer is connected to pin A0.

Libraries/Modules
-----------------

- time Standard Library
    - https://docs.python.org/3/library/time.html
    - Access to the sleep function.
- board CircuitPython Core Module
    - https://circuitpython.readthedocs.io/en/latest/shared-bindings/board/
    - Provides access to the board's GPIO pins and hardware.
- analogio CircuitPython Core Module
    - https://docs.circuitpython.org/en/latest/shared-bindings/analogio/
    - Provides analog hardware support.
- countio CircuitPython Core Module
    - https://docs.circuitpython.org/en/latest/shared-bindings/countio/
    - Provides support for GPIO edge counting.
- digitalio CircuitPython Core Module
    - https://circuitpython.readthedocs.io/en/latest/shared-bindings/digitalio/
    - Provides basic digital pin support.
- pwmio CircuitPython Core Module
    - https://docs.circuitpython.org/en/latest/shared-bindings/pwmio/
    - Provides support for PWM based protocols.
- Adafruit_CircuitPython_Motor CircuitPython Helper Library
    - https://github.com/adafruit/Adafruit_CircuitPython_Motor
    - Provides support for controlling DC motors.

Notes
-----

- Some boards share PWM channels across pins, e.g. pins D9 and D25 on the
  Adafruit Feather RP2040.  Make sure to select different PWM channels for all
  of the motor pins since every pin could be using PWM at the same time.
- Comments are Sphinx (reStructuredText) compatible.

TODO
----

- None.

Author(s)
---------

- Created by John Woolsey on 06-29-2022.
- Modified by John Woolsey on 07-19-2022.

Copyright (c) 2022 Woolsey Workshop.  All rights reserved.

Members
-------
"""


# Imports
from time import sleep
import board
from analogio import AnalogIn
from countio import Counter, Edge
from digitalio import DigitalInOut, Direction, Pull
from pwmio import PWMOut
from adafruit_motor import motor as Motor


# Global Constants
DEBUG = True
"""The mode of operation; `False` = normal, `True` = debug."""

OP_DURATION = 5
"""The operation duration in seconds."""


# Pin Mapping
drv8833_ain1 = PWMOut(board.D9, frequency=50)
"""The PWM enabled pin connected to the AIN1 (motor A control 1) pin of the
DRV8833 motor driver board.  Specifying a PWM frequency of less than 100 Hz
typically improves the low speed operation of brushed DC motors.
"""

drv8833_ain2 = PWMOut(board.D10, frequency=50)
"""The PWM enabled pin connected to the AIN2 (motor A control 2) pin of the
DRV8833 motor driver board.  Specifying a PWM frequency of less than 100 Hz
typically improves the low speed operation of brushed DC motors.
"""

drv8833_bin1 = PWMOut(board.D11, frequency=50)
"""The PWM enabled pin connected to the BIN1 (motor B control 1) pin of the
DRV8833 motor driver board.  Specifying a PWM frequency of less than 100 Hz
typically improves the low speed operation of brushed DC motors.
"""

drv8833_bin2 = PWMOut(board.D12, frequency=50)
"""The PWM enabled pin connected to the BIN2 (motor B control 2) pin of the
DRV8833 motor driver board.  Specifying a PWM frequency of less than 100 Hz
typically improves the low speed operation of brushed DC motors.
"""

drv8833_sleep = DigitalInOut(board.D5)
"""The pin connected to the SLP (sleep) pin of the DRV8833 motor driver board."""

drv8833_fault_counter = Counter(board.D6, edge=Edge.FALL, pull=Pull.UP)
"""The pull-up resistor enabled falling edge pin counter connected to the
FLT (fault) pin of the DRV8833 motor driver board.
"""

pot = AnalogIn(board.A0)
"""The pin connected to the 10 KΩ potentiometer."""


# Global Variables
previous_raw_pot_readings = [0, 0, 0, 0, 0]
"""The last five raw potentiometer readings."""

previous_filtered_pot_reading = 0
"""The previous filtered potentiometer reading."""


# Global Instances
motor_a = Motor.DCMotor(drv8833_ain1, drv8833_ain2)
"""The motor A instance."""

motor_b = Motor.DCMotor(drv8833_bin1, drv8833_bin2)
"""The motor B instance."""


# Functions
def print_motor_status(motor):
    """Prints the status of the specified motor.

    :param motor: The motor instance.
    """

    if motor == motor_a:
        motor_name = "A"
    elif motor == motor_b:
        motor_name = "B"
    else:
        motor_name = "Unknown"
    print(f"Motor {motor_name} throttle is set to {motor.throttle}.")


def basic_operations():
    """Demonstrates the basic operations available within the DCMotor class of
    the Adafruit_CircuitPython_Motor library.
    """

    # Drive forward at full throttle
    motor_a.throttle = 1.0
    if DEBUG: print_motor_status(motor_a)
    sleep(OP_DURATION)

    # Coast to a stop
    motor_a.throttle = None
    if DEBUG: print_motor_status(motor_a)
    sleep(OP_DURATION)

    # Drive backwards at 50% throttle
    motor_a.throttle = -0.5
    if DEBUG: print_motor_status(motor_a)
    sleep(OP_DURATION)

    # Brake to a stop
    motor_a.throttle = 0
    if DEBUG: print_motor_status(motor_a)
    sleep(OP_DURATION)


def check_for_motor_driver_fault():
    """Checks and reports the fault status of the motor driver."""

    if drv8833_fault_counter.count > 0:
        if DEBUG:
            print(f"Motor driver fault(s) detected: {drv8833_fault_counter.count}")
        drv8833_fault_counter.reset()  # reset the fault count back to 0


def ramp_up(motor, direction, duration):
    """Ramps up the speed of a motor from 0% to 100% throttle over a specified
    time period.

    :param motor: The motor instance.
    :param direction: The motor's direction (`forward` or `reverse`).
    :param duration: The length of time, in seconds, to ramp up the speed of
        the motor.
    """

    for speed in [x * 0.01 for x in range(0, 101)]:  # 0.0 to 1.0
        motor.throttle = speed if direction == "forward" else -speed
        sleep(duration / 100)


def ramp_down(motor, direction, duration):
    """Ramps down the speed of a motor from 100% to 0% throttle over a specified
    time period.

    :param motor: The motor instance.
    :param direction: The motor's direction (`forward` or `reverse`).
    :param duration: The length of time, in seconds, to ramp down the speed of
        the motor.
    """

    for speed in [x * 0.01 for x in reversed(range(0, 101))]:  # 1.0 to 0.0
        motor.throttle = speed if direction == "forward" else -speed
        sleep(duration / 100)


def ramping_speed():
    """Ramps up and down the speed of motor A."""

    ramp_up(motor_a, "forward", OP_DURATION)
    ramp_down(motor_a, "forward", OP_DURATION)


def smoothing_filter(current_value):
    """Implements a digital smoothing (average) filter.

    The last 5 values (readings) are averaged to reduce signal noise.

    :param current_value: The current value.

    :return: The filtered value.
    """

    previous_raw_pot_readings[4] = previous_raw_pot_readings[3]
    previous_raw_pot_readings[3] = previous_raw_pot_readings[2]
    previous_raw_pot_readings[2] = previous_raw_pot_readings[1]
    previous_raw_pot_readings[1] = previous_raw_pot_readings[0]
    previous_raw_pot_readings[0] = current_value
    return sum(previous_raw_pot_readings) / len(previous_raw_pot_readings)  # average of raw values


def potentiometer_control():
    """Sets the speed of motor B based on the value retrieved from the
    potentiometer.
    """

    global previous_filtered_pot_reading
    current_raw_pot_reading = pot.value
    current_filtered_pot_reading = smoothing_filter(current_raw_pot_reading)
    if abs(current_filtered_pot_reading - previous_filtered_pot_reading) > 656:  # minimize unnecessary updates, about a 1% change
        if DEBUG: print(f"Potentiometer reading: {current_filtered_pot_reading}")
        motor_b.throttle = current_filtered_pot_reading / 65535  # a value between 0.0 and 1.0
        if DEBUG: print_motor_status(motor_b)
        previous_filtered_pot_reading = current_filtered_pot_reading


def init():
    """Initializes the program."""

    # Sleep pin configuration
    drv8833_sleep.direction = Direction.OUTPUT

    # Motor configurations
    # Specifying slow decay mode may improve spin threshold, speed-to-throttle
    # linearity, and PWM frequency sensitivity of the motors.  The default is
    # fast decay if not specified.
    # motor_a.decay_mode = Motor.SLOW_DECAY
    # motor_b.decay_mode = Motor.SLOW_DECAY

    # Enable (turn on) the motor driver
    drv8833_sleep.value = True


def main():
    """Main program entry."""

    if DEBUG: print("Running in DEBUG mode.  Turn off for normal operation.")
    init()  # program initialization
    while True:
        check_for_motor_driver_fault()
        basic_operations()  # perform basic motor control operations on motor A
        # ramping_speed()  # ramp up and down the speed of motor A
        # potentiometer_control()  # control motor B speed with a potentiometer


if __name__ == "__main__":  # required for generating Sphinx documentation
    main()
