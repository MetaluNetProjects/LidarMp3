# Lidar-MP3

Here is a firmware for Fraise/pico that allows to control the volume
of a looped audio file with the distance of the nearest person.

It uses a GD3300 audio file player module, and a LiDAR RPL-C1 rotating distance sensor.

The LiDAR RPL-C1 is connected to the UART0 through pins 0 and 1.

The GD3300 is connected to the UART1 through pins 4 and 5.

A button is connected to pin 8 and Gnd. This button allows to capture the "background" distance map
(i.e the distance map of the fixed objects around the lidar sensor). The capture is launched 4 seconds after the button is pushed; it lasts 1 second.  
The background map is saved into flash memory, in order to be recalled on next start.

The first audio file found by the GD3300 is automatically looped at startup; the volume depends on the distance of the nearest person: it will start to increase when the person approaches at less than 2 meters, and will be maximum when the person is less than 50 cm away from the sensor.

----------
metalu.net 2024  


