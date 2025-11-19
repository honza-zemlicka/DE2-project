# DE2 line-following robot project

### Team members

* Hana Štěrbová
* Sára Wozarová
* Vojtěch Krehan
* Jan Žemlička

## Project description
This project presents a self-sufficient robot programmed in C, based on the principle of single line following and obstacles detection. 
At the start a calibration is performed in order to correctly differentiate the values between black and white surfaces. The robot follows a proportional line-following method, where one of the two motors is being slowed down depending on the deviation from the line. The deviation(or error) is characterized as how far off the robot is from the center of the line. 
		
Other implemented functions include measuring the distance to an obstacle using an ultrasonic sensor, which allows the robot to adjust its speed or initiate a manuever in order to avoid collision with the obstacle. 
		
The system has integrated RGB sensor for recognizing color markers on the path, enabling the robot to perform special actions on each color.
		
The application also allows user to configurate parametrs such as speed, controller settings or predefined obstacle size. These parameters can be adjusted by rotary encoder and desplayed on a OLED display. The encoder is used for navigation through settings meanwhile the display provides instant visual feedback.
		
To communicate with the hardware, the project uses libraries for GPIO control, timer operations and also for OLED display managing. These modules take care of essential operations needed for the robot to work properly. That includes reading sensor values, controlling the motors, keeping timing accurate and displaying information on the screen.
		
To sum up, this project demonstrates self-operating, C-programmed robot that is capable of adapting its response to different track conditions, obstacles and user-defined parameters.

[**Video demonstration of our project - TBD**]()

![Project poster - TBD]("A3 project poster")

## List of hardware components

* Arduino Nano - main unit

##### Modules

* [HY-SRF05 precision ultrasonic sensor](datasheets/m475c.pdf) - for obstacle distance measurement (may use 2 sensors)
* I2C OLED display 128x32, SSD1306 driver
* [TCS3472 RGB color sensor](datasheets/TCS3472_en.pdf) - for color mark recognition on the track

##### Components

* [L293D](datasheets/l293d.pdf) - driver used for engine control
* DC motor N20 (500rpm/6V) - 2x
* IR LED diode - as an IR transmitter - 4x
* IR phototransistor - as an IR receiver - 4x
* buttons, stabilizer, connectors, batteries ...

## Software design

TBD (block diagrams, flowcharts, pseudocodes, library documentations...)

## References

- https://github.com/tomas-fryza/avr-labs
