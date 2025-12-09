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

![Block diagram](images/block%20diagram.png "Block diagram")
> Hardware block diagram

* Arduino Nano - main controller unit

##### Modules

* [HY-SRF05 precision ultrasonic sensor](datasheets/m475c.pdf) - for obstacle distance measurement (may use 2 sensors)
* I2C OLED display 128x32, SSD1306 driver

##### Components

* [L293D](datasheets/l293d.pdf) - driver used for engine control
* DC motor N20 (500rpm/6V) - 2x
* IR LED diode - as an IR transmitter - 4x
* IR phototransistor - as an IR receiver - 4x
* stabilizer, buttons, connectors, batteries ...

## Software design

<div align="center">
    <img src="images/main%20loop%20flowchart.png" width="30%">
    <br>
    <em>Main loop flowchart</em>
</div>

The program initializes peripherals (ADC, I2C, PWM) and runs calibration before entering an infinite control loop that reads analog sensors to calculate deviation from track and drive motors via PWM. It also handles periodic ultrasound & RGB tasks. Upon pressing a button, the robot main loop enters stopped state to pause and resume execution.

<div align="center">
<img src="images/calibration%20flowchart.jpg" width="30%">
    <br>
    <em>Calibration loop flowchart</em>
</div>

This routine clears sensor variables and enters a loop to continuously sample IR sensors values, updating the minimum and maximum thresholds. The calibration process is looped until a manual button press confirms completion, turning off the status LED and returning to the main loop.

### Doxygen documentation 

[Open Documentation](Doxygen_html/index.html)

## References

- https://github.com/tomas-fryza/avr-labs
- https://miro.com/
- https://chatgpt.com/
- https://gemini.google.com/
