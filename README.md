# DE2 line-following robot project

### Team members

* Hana Štěrbová
* Sára Wozarová
* Vojtěch Krehan
* Jan Žemlička

## Project description
This project presents an autonomous robot designed for single-line following and obstacle avoidance. On startup, the robot performs a calibration routine to distinguish between black and white surfaces in current lighting conditions. It utilizes a proportional control algorithm to follow the line by adjusting the speed of individual motors based on robot's deviation from the followed path.
		
An ultrasonic sensor measures the distance from obstacle, allowing the robot to initiate an dodge maneuver to avoid collision.
		
Users can configure parameters as robot speed and control gain via remote IR controller and set values are displayed on an OLED display.

The robot’s firmware, written in C, utilizes libraries from [DE2 AVR course](https://github.com/tomas-fryza/avr-labs) – gpio, oled & twi libraries (+ uart for troubleshooting). Additionally, an ultrasound library was created to handle sensor's initialization & distance calculation.

[**Video demonstration of our project**](https://youtu.be/tJKh0oyu3Gs?si=n_vKxXadf_YJ9QNw)

![Project poster](images/poster.png "A3 project poster")

## Hardware description

<div align="center">
    <img src="images/block%20diagram.png">
    <br>
    <em>Hardware block diagram</em>
</div>

* [Arduino Nano](datasheets/ArduinoNANO-datasheet.pdf) - main controller unit

##### Modules

* [HY-SRF05 precision ultrasonic sensor](datasheets/m475c.pdf) - for obstacle distance measurement (may use 2 sensors)
* I2C OLED display 128x32, SSD1306 driver

##### Components

* [L293D](datasheets/l293d.pdf) - driver used for engine control
* DC motor N20 (500rpm/6V) - 2x
* [IR333C/H0/L10](datasheets/IR_LED_ir333c.pdf) - IR sensor transmit - 4x
* [OS-1838B](datasheets/IR_receiver_vs1838b.pdf) - IR sensor receiver - 4x
* [OS-1838B](datasheets/IR_receiver_vs1838b.pdf) - IR remote control receiver
* stabilizer, buttons, connectors, batteries ...

## Software design

<div align="center">
    <img src="images/main%20loop%20flowchart.png" width="30%">
    <br>
    <em>Main loop flowchart</em>
</div>

The program initializes peripherals (ADC, I2C, PWM & ultrasound) and runs calibration routine before entering an infinite control loop that reads IR sensors values to calculate deviation from track and drive motors via PWM. It also handles periodic ultrasound task. Upon pressing a button, the robot main loop enters stopped state to pause and resume main loop execution.

<div align="center">
<img src="images/calibration%20flowchart.jpg" width="30%">
    <br>
    <em>Calibration loop flowchart</em>
</div>

This routine clears sensor variables and enters a loop to continuously sample IR sensors values, updating the minimum (black) and maximum (white) thresholds. The calibration process is looped until a manual button press confirms completion, turning off the status LED and returning to the main loop.

[### Doxygen Documentation](https://honza-zemlicka.github.io/DE2-project/Doxygen_html/index.html)

## References

- https://github.com/tomas-fryza/avr-labs
- https://miro.com/
- https://chatgpt.com/
- https://gemini.google.com/
