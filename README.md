# Blink-GoPro-GPS-Arduino
Interface and trigger your GoPro using the CamDo Blink camera controller, an Arduino and GPS module.
Complete with an LCD display this is a fun and straight forward project. The CamDo Blink enables remote control of your GoPro via the motion detector input. But instead of a motion detector, we instead use the Arduino microcontroller to trigger it in response to the GPS position and associated calculation. 

There are plenty of applications for this, basically any of the data that a GPS can generate (or that you can infer / calculate) can be used, including:

* Take a photo when I am in proximity to a certain landmark
* Take a photo every x seconds when I am travelling above or below a certain speed
* Take a photo every x metres (the example we have implemented in this project)

The basic setup is as follows:
![GoPro GPS Schematic](https://cdn.shopify.com/s/files/1/0906/7602/files/GPS_Arduino_Microcontroller_and_Blink_interface_schematic.jpg?v=1503906297)

Parts required:
* 1 x Arduino/Genuino UNO R3 (genuine) / (replica) or similar
* GoPro camera (HERO4 or HERO3+Black)
* 1 x [CamDo Blink time lapse and camera controller](https://cam-do.com/products/blink-gopro-time-lapse-controller)
* 1 x 16x2 Arduino LCD Keypad Shield
* 1 x UBlox GPS Module or similar (DFRobot has one with a case)
* 3.5mm TRS audio stereo cable
* 5V power supply eg a V15 or V44 battery if you are going hiking.

For all of the detailed instructions visit [https://cam-do.com/blogs/camdo-blog/how-to-trigger-your-gopro-using-gps-arduino-microcontroller](https://cam-do.com/blogs/camdo-blog/how-to-trigger-your-gopro-using-gps-arduino-microcontroller)

Also thanks to Mikal Hart and his TinyGPS library. V1.3 is included in this project. [Visit Mikal's Github page here.] (https://github.com/mikalhart/TinyGPS)
