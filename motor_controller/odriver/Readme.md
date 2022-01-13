### Pre-requisities

1. Install odrivetool on your computer, Ubuntu is recommended. Follow [ODrive instruction](https://docs.odriverobotics.com/#downloading-and-installing-tools).

2. Erase the previous configurations: (type or paste the line below in odrivetool console)
```
odrv0.erase_configuration()
odrv0.save_configuration()
```

### Prepare ODrive:
1. make sure you have the proper firmware on the board (v3.6 board, firmware v0.4.11 .. v0.5.1 are tested conbination as of Jan 2022)
  - https://docs.odriverobotics.com/odrivetool#device-firmware-update
  - make sure the switch (DFU-RUN) near the usb connect is RUN
    - You can see like `Bus 001 Device 017: ID 1209:0d32 InterBiometrics` with `lsusb` command if it is RUN
    - You can see like `Bus 001 Device 007: ID 0483:df11 STMicroelectronics STM Device in DFU Mode` with `lsusb` command if it DFU
2. set ASCII mode enabled with odrivetool if you use ASCII mode (serial protocol)
  ```
  odrv0.config.enable_ascii_protocol_on_usb = True
  odrv0.save_configuration()
  odrv0.reboot()
  ```
  
  - you can see like `ttyACM0` with `ls /dev` command
  - you can also see `ttyODRIVE` if you have run [setup-usb.sh](https://github.com/CMU-cabot/cabot-docker/blob/master/setup-usb.sh)
    
    
### Configure ODrive

1. Set up new configurations, just paste the lines altogether in ODrivetool terminal:
```
odrv0.axis0.motor.config.current_lim = 45
odrv0.axis1.motor.config.current_lim = 45
odrv0.axis1.motor.config.calibration_current = 30
odrv0.axis0.motor.config.calibration_current = 30
odrv0.axis0.encoder.config.use_index = 1
odrv0.axis1.encoder.config.use_index = 1
odrv0.axis1.controller.config.vel_limit = 370000
odrv0.axis0.controller.config.vel_limit = 370000
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.circular_setpoints = 1
odrv0.axis1.controller.config.circular_setpoints = 1
```
(Read [ODrive documentation](https://docs.odriverobotics.com/) to understand more about configurations.)

2. Save the configurations:
```
odrv0.save_configuration()
```

3. Reboot ODrive to make sure the configuration is saved:
```
odrv0.reboot()
```

4. Full calibration for both motors and encoders
**Do calibrate one by one**
```
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```
You hear beep sound and motor will rotate. Do another after it stoped.
```
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

5. Set calibration-related parameter:
```
odrv0.axis0.encoder.config.pre_calibrated = 1
odrv0.axis0.motor.config.pre_calibrated = 1
odrv0.axis1.encoder.config.pre_calibrated = 1
odrv0.axis1.motor.config.pre_calibrated = 1
``` 

6. Save and reboot:
```
odrv0.save_configuration()
odrv0.reboot()
```

7. Adjust PID
https://docs.odriverobotics.com/control
Tune PI controller for velocity control mode: (using axis0 as example)

---
**pre tuned value with D5065 KV270 motor with AMT102-V encoder (8192 count)**
one example (E2)
```
<axis>.controller.config.vel_gain = 0.002                (default value = 0.0005)
<axis>.controller.config.vel_integrator_gain = 0.015     (vel_gain * 0.5 * 15Hz, default value = 0.001)
<axis>.encoder.config.bandwidth = 1000                   (default)
```
another setting (GT)
```
odrv0.axis0.motor.config.current_lim = 30
odrv0.axis0.motor.config.current_lim_margin = 15
odrv0.axis0.motor.config.torque_constant = 0.0306              # (8.27 / 270)
odrv0.axis1.motor.config.current_control_bandwidth = 200
odrv0.axis1.motor.config.current_lim = 30
odrv0.axis1.motor.config.current_lim_margin = 15
odrv0.axis1.motor.config.torque_constant = 0.0306              # (8.27 / 270)

# latest version of odriver sets tuned parameter sets at startup
motor.config.current_control_bandwidth = 200
encoder.config.bandwidth = 200

# these parameters' unit are changed
# need to multiply 450 (torque_constant * count_per_round) to the previous param
controller.config.vel_gain = 4.5                   # [Nm/(turn/s)]
controller.config.vel_integrator_gain = 45         # [Nm/((turn/s) * s)]
```
---

  - Set vel_integrator_gain to be zero:
```
odrv0.axis0.controller.config.vel_integrator_gain = 0
```

  - Turn on loop control for this axis:
```
odrv0.axis0.requested_state = 8
```
  - If there is vibration in the system, reduce vel_gain until vibration stops; if there isn’t, skip this step:
```
odrv0.axis0.controller.config.vel_gain = xx(your value)
```
  - Keep increasing vel_gain by 10% until you see vibration. **When you increase the value, try to turn/tap the wheel to intentionally initiate vibration on motor in order to confirm that no vibration happens with the value.** You may be able to increase up to much higher value with static setting
  - Increase encoder bandwidth can reduce vibration while increasing vel_gain. Too high value causes error in control, it should be less than 3000 with CUI AMT102-V encoder.
```
odrv0.axis0.encoder.config.bandwidth = 1000~3000 (CUI AMT102-V)
```  
  - Set the vel_integrator_gain to be 0.5*control_frequency*vel_gain. For example, if the velcocity is updated 10 times every second (10Hz), vel_integrator_gain would be 0.5*10*vel_gain:
```
odrv0.axis0.controller.config.vel_integrator_gain = xx(your value)
```

  - Set the velocity to 50% and 100% of the max velocity that might be used in your system, and go back to step 3 and play with the two gain value. (You can try 25% and 75% or more different velocity as well to evaluate the overall performance.)

  - Try to increase vel_integrator_gain by 50% or even more to have a more stiff torque control.

  - Save and reboot:
```
odrv0.save_configuration()
odrv0.reboot()
```
  - Try with load and check noise
