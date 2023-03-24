# Triskarino
SW and applications on Triskarinos

This directory contains the basic SW and stl files common to all Triskarinos, the omnidirectional robot basis developed at AIRLab-POLIMI.

- ArduinoMega2.0.zip contains a sample file running on the microcontroller of the Triskarino, typically an Arduino Mega, to control the motors and detect distances from sonars. A python program is also present to show how to drive teh motors from a high-level computer (on most Arduinos we have either a Raspberry Pi 4, or a NVIDIA Nano
- libraries.zip contains the libraries used by the sample file, in particular: NewPing, for sonars, VIRHAS (developed by Michele Bertoni), to control via PID the motors starting from frontal speed, angular speed and strafe, Cytron_Motor_Drivers_Library to drive the Cytron motor drivers, Encoder to read the motor encoders, Adafruit_Neopixel, to ocntrol the neopixels, and MR001004 to use the DualMC33926MotorShield, still present on some models.
- An stl file to print the connectors from motors to omniwheels


