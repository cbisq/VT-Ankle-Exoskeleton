# Chance Bisquera
# TMVS Lab: Ankle Exoskeleton Thesis
# 6/19/21
# O-Drive Preliminary Testing Script

"""
This Python script is meant to set the start-up configuration settings for an O-Drive BLDC motor controller in a single compact 
file. 

The following lines of code were written in accordance with the ODrive configuration instructions available at:
https://docs.odriverobotics.com/
https://docs.odriverobotics.com/hoverboard

This configuration script also drew inspiration from ODrive Hoverboard configuration code written by Austin Owens. The original
code and YouTube tutrial video written by Mr. Owens is available via the following links:
https://www.youtube.com/watch?v=9UxTPxgvOAA&list=LL&index=1&t=27s
https://github.com/AustinOwens/robodog/blob/main/odrive/configs/odrive_hoverboard_config.py

"""

import sys
import time
import odrive
from odrive.enums import *
from fibre.protocol import ChannelBrokenException

"""Start-up Script for the ODrive """

# Check the Main Supply Voltage at Start-Up
odrv0.vbus_voltage

# Erase Previous Configuration Settings (May be Optional)
odrv0.erase_configuration()

# Set Motor Configuration Parameters
odrv0.axis1.motor.config.pole_pairs = 21 # Specified on T-Motor datasheet
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4
odrv0.axis1.motor.config.requested_current_range = 14
odrv0.axis1.motor.config.current_control_bandwidth = 1000 # 100 may be recommended for Hoverboard motors (default 1000)
odrv0.axis1.motor.config.torque_constant = 0.0868 # Specified on T-Motor datasheet [Nm/A]

# Set the Hall Sensor Configuration Parameters
odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = 6*odrv0.axis1.motor.config.pole_pairs # Hall feedback has 6 states for every pole pair in the motor
odrv0.axis1.encoder.config.calib_scan_distance = 150 # Need to increase from default given low resolution of Hall sensors
odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL
odrv0.axis1.encoder.config.bandwidth = 100 # Need to reduce from default of 1000 given low cpr of Hall sensors

# Configure Controller Gains
# ODrive PID Control Diagram Available at: https://docs.odriverobotics.com/control
odrv0.axis1.controller.config.pos_gain = 1
odrv0.axis1.controller.config.vel_gain = 0.02 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr
odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr
odrv0.axis1.controller.config.vel_limit = 18

# Enable the ODrive's UART_A Interface
odrv0.config.enable_uart_a
#odrv0.config.enable_uart # Command doesn't seem to work
#odrv0.config.gpio1_mode = GPIO_MODE_UART_A
#odrv0.config.gpio2_mode = GPIO_MODE_UART_A

# Enable the Brake Resistor
odrv0.config.enable_brake_resistor = True

# Set Controller Mode to Position Control
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

# Save Configuration Settings and Reset the ODrive
print("Saving motor configuration settings and rebooting...")
odrv0.save_configuration()
#odrv0.reboot()

# Initiate Motor Calibration
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION # You should hear the motor beep at this step
odrv0.axis1.motor # Check phase and inductance values
odrv0.axis1.motor.error # Check for any present errors

"""
***R80 T-Motor Specifications***
Resistance Phase to Phase = 125 mOhms = 0.125
Inductance Phase to Phase = 87 uH = 0.000087
Available: https://store-en.tmotor.com/goods.php?id=944
"""

# If calibration data looks good, save calibration settings to persistent memory
calibrationVerify1 = raw_input("Please enter 'True' if settings look good.")

if(calibrationVerify1 == 'True'):
	odrv0.axis1.motor.config.pre_calibrated = True
	odrv0.save_configuration()
	print('Precalibration settings saved to persistent memory. Checking motor and Hall sensor alignment...')

else:
	print('ODrive settings are not satisfactory. Please fix the settings and try again.')
	quit()

# Initiate Encoder Calibration
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION # Added 9/5/21 (works if 22 nF capacitors are included)
#odrv0.axis1.encoder.config.hall_polarity_calibrated = True # Recommended fix/alternative to the above command (recommended as a work-around by ODrive technical support) [not needed if 22 nF capacitors were added]
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.encoder
odrv0.axis1.encoder.error # Check to see if there are any errors

"""
Tip from ODrive:
Check that there are no errors. If your hall sensors has a standard timing angle then offset_float should 
be close to 0.5 mod 1. Meaning values close to -1.5, -0.5, 0.5, or 1.5, etc are all good.
"""

# Save calibration to persistent memory of no errors are present and calibration looks good
calibrationVerify2 = raw_input("Please enter 'True' if there are no errors in Hall sensor calibration.")
if(calibrationVerify2 == 'True'):
	odrv0.axis1.encoder.config.pre_calibrated = True

else:
	print('Offset settings are not satisfactory. Please fix the settings and try again.')
	quit()

# Save encoder configuration settings and reset the ODrive
print("Saving encoder configuration settings and rebooting...")
odrv0.save_configuration()
odrv0.reboot()

print("Configuration Settings Completed!")

# Run Spin Test on the Motor
print("Conducting spin test on motor.")
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.input_pos = 2
# Your motor should spin here
print("Test complete.")

# Reset the ODrive to its idle state to await further instructions
odrv0.axis1.requested_state = AXIS_STATE_IDLE
print("ODrive set to Idle Mode.")
print("Awaiting further instructions.")

# ODrive PID Gains (Default)
odrv0.axis1.controller.config.pos_gain = 1
odrv0.axis1.controller.config.vel_gain = 0.02 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr
odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * odrv0.axis1.motor.config.torque_constant * odrv0.axis1.encoder.config.cpr

# ODrive PID Gains (Experimental)
odrv0.axis1.controller.config.pos_gain = 
odrv0.axis1.controller.config.vel_gain = 
odrv0.axis1.controller.config.vel_integrator_gain = 

""" Filtered Position Control """
odrv0.axis1.controller.config.input_filter_bandwidth = 2.0 # Units: [1/s]
odrv0.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER

odrv0.axis1.controller.input_pos = 1 # [turns]

""" Trajectory Control """
# Trajectory Control Configuration Parameters
odrv0.axis1.motor.config.current_lim = 14 # Units: [A]
odrv0.axis1.motor.config.current_lim_margin = 8 # Units: [A]
odrv0.axis1.trap_traj.config.vel_limit = 15 # Units: RPS
odrv0.axis1.trap_traj.config.accel_limit = 17
odrv0.axis1.trap_traj.config.decel_limit = 17
odrv0.axis1.controller.config.inertia = 0 # Inertia of load not presently known

# Set Trajectory Control Mode and Move the Motor
odrv0.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
odrv0.axis1.controller.input_pos = 2

# Code for creating a live plot of variables of interest
start_liveplotter(lambda:[odrv0.axis1.encoder.pos_estimate, odrv0.axis1.controller.pos_setpoint])

odrv0.axis1.motor.current_control.Iq_measured

# Code for creating a live plot of the Hall Sensors
start_liveplotter(lambda:[odrv0.axis1.encoder.hall_state])

# Code for creating a live plot of motor current
#start_liveplotter(lambda:[odrv0.axis1.motor.current_control.Iq_measured, odrv0.axis1.motor.current_control.Iq_setpoint])
start_liveplotter(lambda:[odrv0.axis1.motor.current_control.Iq_setpoint, odrv0.axis1.motor.current_control.Iq_measured])

# Code for creating a live plot of motor speed
start_liveplotter(lambda:[odrv0.axis1.encoder.vel_estimate, odrv0.axis1.controller.vel_setpoint])

# Code for creating a live plot of the regenerative current
start_liveplotter(lambda:[odrv0.ibus])

"""Debugging Tips """
dump_errors(odrv0, True) # Prints all present errors on the ODrive

axis0.encoder.shadow_count # Shadow_count tracks encoder motion, even before the encoder or motor are calibrated. If your encoder is working, you should see this value change when you turn the motor.


""""Code for Backing Up a Configuration"""
odrivetool backup-config my_config.json 	# Save Configuration to a file on the PC
odrivetool restore-config my_config.json 	# Restore the Configuration from a File
# Note: "Note that encoder offset calibration is not restored because this would be dangerous if you transfer the calibration values of one axis to another axis"
