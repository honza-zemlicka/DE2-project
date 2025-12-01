# DE2 line-following robot project

### Team members

* Hana Štěrbová
* Sára Wozarová
* Vojtěch Krehan
* Jan Žemlička

## Project description
This project presents an autonomous robot designed for single-line following and obstacle detection. Upon startup, the robot performs a calibration routine to distinguish between black and white surfaces in the current lighting conditions. It utilizes a proportional control algorithm to follow the line by adjusting the speed of individual motors based on the robot's deviation from the followed path.
		
An ultrasonic sensor measures the distance from obstacles, allowing the robot to adjust its speed or initiate maneuvers to avoid collisions. Additionally, an integrated RGB sensor detects color markers on the path, enabling specific actions based on the color recognized.
		
Users can configure parameters such as robot speed, controller settings, and obstacle thresholds via a rotary encoder and an OLED display. The encoder is used for menu navigation, while the display provides visual feedback.

The robot’s firmware, written in C, utilizes libraries for GPIO control, timer operations, and display management. These modules handle essential tasks such as sensor data acquisition, motor control, and system timing.

[**Video demonstration of our project - TBD**]()

![Project poster - TBD]("A3 project poster")

## Hardware description

![Block diagram](images/block%20diagram.jpg "Block diagram")
> Hardware block diagram

* Arduino Nano - main controller unit

##### Modules

* [HY-SRF05 precision ultrasonic sensor](datasheets/m475c.pdf) - for obstacle distance measurement (may use 2 sensors)
* I2C OLED display 128x32, SSD1306 driver
* [TCS3472 RGB color sensor](datasheets/TCS3472_en.pdf) - for color mark recognition on the track

##### Components

* [L293D](datasheets/l293d.pdf) - driver used for engine control
* DC motor N20 (500rpm/6V) - 2x
* IR LED diode - as an IR transmitter - 4x
* IR phototransistor - as an IR receiver - 4x
* stabilizer, buttons, connectors, batteries ...

## Software design

![Main loop](images/main%20loop%20flowchart.jpg "Main loop flowchart")
> Main loop flowchart

![Calibration](images/calibration%20flowchart.jpg "Calibration flowchart")
> Calibration loop flowchart

## References

- https://github.com/tomas-fryza/avr-labs
- https://miro.com/
- https://chatgpt.com/
- https://gemini.google.com/