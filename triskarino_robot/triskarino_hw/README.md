# Triskarino Hardware (triskarino_hw)

This folder contains the nodes to interact with triskarino's HW (Wheels, LED, Speaker, Sonar, Touch Sensors, Lidar, Camera and IMU) and to read joystick commands

## Base, Sonar, LEDS, Touch Sensors

Base, Sonar, LED and Touch Sensors are all connected either to Arduino or to ESP boards. Both Arduino and ESP communicate to the Jetson through the arduino ros library, meaning they can read published topics and can publish topics of their own. Specifically,:
- The base read command velocities (cmd_vel_out_filled topic) and move accordingly. 
- The LEDs read light commands (light topic) and do the corresponding light animation
- The base also publishes sonar (sonar topic) and odometry of the wheels (rawOdometry topic)
- TouchSensors also publish touch data

To make basically everything work it is necessary to insert the paths of the serial into the various configuration files. For the base it goes into arduino_param.yaml, for the lights at the moment is directly into the various launch files (es. sound_light.launch), for the touch sensors in touch_mega.yaml and touch_param.yaml.

## More on the base

To allow more than one control of the base at the same time (i.e. joystick, autonomous) all the velocities published go through a multiplexer (twist_mux) before going to the base. The multiplexer decides on which command to send based on the priority specified in the file in config/twist_mux.yaml. 
Also, the velocities coming out from the multiplexer are read and re-published at a fixed rate by the speed manager which are then read by the Arduino connected to the motors. Re-publishing velocities at a fixed rate, allows the PID of the base to follow the target more smoothly. 

## LIDAR

To access the Lidar we use the urg_node (publicly available ros node), which constantly publishes lidar data on the scan topic. To filter out readings that come from the specific angles of screws that block the Lidar we use the ros laser filter node. All of this is regulated with the files in the configs folder lidar_config.yaml (here also it is necessary to insert to path of the serial of the Lidar es. /dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00) and laser_filter.yaml. To tune laser_filter and check if the Lidar works, the rviz launch in triskarino_description can be used. 

## IMU

To access the IMU we use the publicly available mpu6050_driver node. The IMU is directly connected to the Jetson so it should work out of the box.

## Joystick

The jostick is also connected by serial, the serial path is included directly into the teleop_manager.py. Instruction on which button does what are directly commented into the teleop_manager.py file.
For now:
- A,B,X,Y do lights of different colors
- the arrows do different sounds
- the right stick makes the robot move forward and strafe
- pressing the right stick makes the robot stop
- the LT and RT buttons (the back ones) make the robot rotate



