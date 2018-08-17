# GyroAngle-Code_Arduino

This is the setup to get data from Gyro in a phone, to make bots go straight.
The angle is a float value sent to an Arduino connected with phone on the serial port.
this Arduino will read the value, convert it to I2C packets, and transfer those packets to another Arduino, 
which is controlling the motion and getting directions as well from the controller

The controller interface is FlySky FS-iA6 RC TX, running the 14CH mod firmware,with IBUS protocol for data. 
