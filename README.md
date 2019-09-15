# TurtleFeeder - Arduino code and information on how to modify a dobar or SaiDeng automatic fish pond feeder
(C) Daniel Kirstenpfad 2019 - https://www.schrankmonster.de

## Requirements

 - an ESP8266 based Arduino compatible module - Wemos D1 Mini recommended
 - Arduino IDE 1.8.8
 - Arduino ESP8266 library 2.5.2 - https://github.com/esp8266/Arduino
 - Arduino MQTT Library 2.4.1 by 256dpi - https://github.com/256dpi/arduino-mqtt

This also uses the Arduino OTA functionality. So when the ESP8266 successfully connected to your Wifi you will
be able to use the Arduino IDE over-the-air functionality.

## Configuration

What you need to configure:
 - the output pins you have chosen - D1+D2 are pre-configured
 - WiFi SSID + PASS
 - MQTT Server (IP(+Username+PW))
 - MQTT Topic prefix

Commands that can be sent through mqtt to the /feed topic.

## MQTT topics and control
There are overall two MQTT topics:
   (1) $prefix/feeder-$chipid/state
       This topic will hold the current state of the feeder. It will show a number starting from 0 up.
       When the feeder is ready it will be 0. When it's currently feeding it will be 1 and up - counting down for
       every successfull turn done.
       There is an safety cut-off for the motor. If the motor is longer active than configured in the MaximumMotorRuntime
       variable it will shut-off by itself and set the state to -1.

   (2) $prefix/feeder-$chipid/feed
       This topic acts as the command topic to start / control the feeding process. If you want to start the process
       you would send the number of turns you want to happen. So 1 to 5 seems reasonable. The feeder will show the
       progress in the /state topic.
       You can update the amount any time to shorten / lengthen the process.
       On the very first feed request after initial power-up / reboot the feeder will do a calibration run. This is to make
       sure that all the wheels are in the right position to work flawlessly.

## examples (for feeder-00F3B839):

mosquitto_pub -t house/stappenbach/feeder/feeder-00F3B839/feed -m 3
-> will feed 3 times

mosquitto_sub -v -t house/stappenbach/feeder/feeder-00F3B839/state
-> will show the current state of the feeder
