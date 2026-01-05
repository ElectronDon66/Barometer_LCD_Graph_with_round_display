This project repository contains a schematic , Bill of Material ,picture and Arduino code for a graphing barometer.
The project has two displays:
A 320 x 240 ILI9341 TFT LCD display that shows the last 10 hours of the barometric pressure (2.8 inch diagonal) 
A round GC9A01 (1.28 inch round) LCD display 
The processor used is a Teensy 4.0 .
 The air pressure sensor is an Adafruit BMP390. A potentiometer allows you to calibrate the barometric pressure reading to a local airport or weather station. 
The potentiometer is creating an analog voltage read by the Teensy 4.0 that is proportional to your altitude (above sea level) offset as all barometers
are typically set to read the barometric pressure as if you location was located at sea level. To set the pot,  power up the circuitry and watch the round 
current barometric pressure reading. Obtain your local barometric pressure (cell phone weather ) or local airport altimeter setting. rotate the pot until the 
needle on the round display is the same as the local barometric pressure and you are done.
The round display updates once a second. The graphing LCD takes data every 6 minutes, but only updates the display once an hour. Upon initial powerup  the  graphing
display takes the current pressure reading and populates the whole display with just the current reading. 
Pressure data is in inches of mercury (In/Hg). 
The graphing display shows relative pressure changes based on the average pressure with a 10 point average. This is so you can see small pressure changes.
The round display is absolute pressure so it extends from 28 In/Hg up to 32 In/Hg. 

I used Arduino's IDE to write the code. In order to run the code on a teensy 4.0 microcontroller you'll  also need to install the Teensy URL in the Arduino board manager (preferences).Probably the easiest way for you to do this is go to https://www.pjrc.com/teensy/tutorial.html and follow the instructions to load the Teensyduino into the Arduino boards manager. This is a critical step to use the Teensy. Essentially you'll be opening the Arduino board manager folder, searching for Teensy  by Paul Stoffregen and clicking install. The tutorial at the PJRC site will help you to do this. Then don't forget to go to Arduino IDE and select the Teensy 4.0 under the tools tab.  There are also some youtube videos that show how to do this but it's not all that difficult. 


