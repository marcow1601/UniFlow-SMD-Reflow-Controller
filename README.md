# UniFlow SMD Reflow Controller

## Introduction

This software is designed to run the process of at-home SMD circuit board (PCB) reflow soldering in an easy, customizable, precise and reproducible manner. This requires accurate realtime data and algorithms to follow a preset temperature profile as closely as possible.

An ESP8266 microcontroller is the brain of the operation. Process parameters can easily be adjusted by using the nice, rotary encoder based user interface. The current settings as well as all required live process data is being displayed on a high-contrast OLED display.

During the reflow cycle, the current temperature of the reflow device is regularly being acquired via a thermocouple sensor, capable of precisely reading temperatures of up to 1.000&deg;C. According to the currently set process parameters, a software PID-controller decides on how to reach the setpoint temperature in the most accurate way.

By PWM switching a solid state relay (SSR), connected to the heating element of the reflow device - no matter if it is a custom hotplate, a torn down toaster oven, an old clothing iron or a dedicated reflow oven - the heating power can precisely be adjusted to the current requirements.


## Features

* ESP8266 microcontroller platform
* MAX6675 thermocouple readout
* Dual PID-controller for temperature and slope setpoints
* High-contrast OLED display
* Easy to use rotary encoder interface


## Hardware platform

The software has been optimized to run on a custom made hardware platform called **UniFlow**.

UniFlow is a completely ready-to-go solution for everything and anything requiring temperature control (reflow soldering, sous vide cooking, pizza oven,...). It comes with the following features:

- [x] ESP8266 microcontroller
- [x] 0.96" OLED display
- [x] Rotary encoder
- [x] MAX6675 thermocouple converter
- [x] Relay output
- [x] 2 SK6812 RGBW status LEDs
- [x] USB-C programming port
- [x] CP2102 auto-program circuit
- [x] 4 general purpose switches
- [x] 4 general purpose GPIOs

The final version of UniFlow will soon be available for purchase here: ~~UniFlow in my Tindie Store~~

<a href="https://www.tindie.com/stores/marcosprojects/?ref=offsite_badges&utm_source=sellers_marcosprojects&utm_medium=badges&utm_campaign=badge_medium" target="_blank"><img src="https://d2ss6ovg47m0r5.cloudfront.net/badges/tindie-mediums.png" alt="I sell on Tindie" width="150" height="78"></a>


## Library dependencies and drivers

To compile this software using the Arduino IDE the following non-standard libraries will be required:

* Adafruit SSD1306 ([Adafruit on GitHub](https://github.com/adafruit/Adafruit_SSD1306))
* Adafruit GFK ([Adafruit on GitHub](https://github.com/adafruit/Adafruit-GFX-Library))
* Adafruit Neopixel ([Adafruit on GitHub](https://github.com/adafruit/Adafruit_NeoPixel))
* Adafruit MAX6675 with a modification explained [here](https://github.com/adafruit/MAX6675-library/issues/9) ([Adafruit on Github](https://github.com/adafruit/MAX6675-library))
* MCP23017_WE ([wollewald on GitHub](https://github.com/wollewald/MCP23017_WE))
* Arduino PID library ([br3ttb on GitHub](https://github.com/br3ttb/Arduino-PID-Library))

In order to have full functionality of the USB-C port, the following SiLabs CP2102 driver will be required:
* SiLabs CP210x USB to UART VCP driver download ([SiLabs downloads page](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers))
