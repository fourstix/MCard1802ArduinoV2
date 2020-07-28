MCard1802ArduinoV2
==================

This repository contains the second version of Arduino programs for supporting the 1802 Membership Card with Arduino based hardware.

My first computer was a Netronics Elf II. It was an RCA 1802 based single board computer that was sold as a kit,
but it had a hexadecimal keypad, an LED, video and an expansion bus. Like many high school kids, I mowed yards
and spent my hard-earned dollars to buy a kit from Netronics [based on their ads.](http://www.cosmacelf.com/gallery/netronics-ads/)

My orignal Elf II was lost in a move long ago, but today Lee Hart's 1802 Membership card duplicates the orignal elf hardware.
However, instead of the orignal Elf front panel, I wanted to simulate the Netronics Elf II interface using hardware I had at hand. 
I wrote Arduino based code to support communication to the 1802 Membership card and to simulate the Netronics Elf. I also created
code to simulate the original Pixie video as well.

Description
------------

This code is the second version of the [MCard1802Arduino](https://github.com/fourstix/MCard1802Arduino) code using a 16 x 2 LCD 
character display and Teensy 3.2 for Pixie Video.  In this version the hardware and code design is a bit cleaner with the hardware
divided into two cards, a Front Panel Card and a Daughter Card.

The Front Panel Card consists of an MCP23008 I2C 8 bit port expander to drive the 1802 Control lines,
a 7400 Quad Nand logic chip for Write Enable and Serial Communication logic, and an MCP23017 I2C dual port IO expander to communicate
with the 1802 Membership Card's data in and data out lines.  The Daughter Card provides the video and address line logic along with
support for the MCSMP20J ROM. A minimum implementation with an Arduino can be done with the Front Panel Card alone.

Introduction
-------------

A very good source of information on the RCA 1802 chip and Cosmac Elf computer can be found on the 
[CosmacElf web page.](http://www.cosmacelf.com) The 1802 was a fantastic microprocessor that still has quite a 
dedicated following today.

The 1802 Membership card is available from Lee Hart on his [website.](http://www.sunrise-ev.com/1802.htm)  
Additional documentation and other information are availble from Herb Johnson's 
[1802 Membership Card wesite.](http://www.retrotechnology.com/memship/memship.html)

This code supports an I2C 16 x 2 LCD as the display.  This code has been tested with the 
[Sparkfun SERLCD display](https://www.sparkfun.com/products/14073) and a generic I2C based 1602 LCD display.
One could also use the [Adafruit LCD backpack](https://www.adafruit.com/product/292) for an I2C based
16 x 2 LCD display.
 
A Qwiic hexadecimal keypad created from a 4x4 keypad and a 3.3v/8MHz Arduino pro-mini is used for input.
Details about the hexadecimal keypad including firmware are available at the 
[fourstix/Hex_Keypad_Arduino_Library](https://github.com/fourstix/Hex_Keypad_Arduino_Library)
git hub repsository.

The Sparkfun [Qwiic](https://www.sparkfun.com/qwiic) interface is a 3.3V I2C based interface that makes
it very easy to connect various hardware to the Arduiono.  

Information on the Sparkfun Qwiic interface is available [here.](https://www.sparkfun.com/qwiic)

The Daughter Card simulates a Cdp1861 Pixie Video chip, using a [Teensy 3.2.](https://www.pjrc.com/teensy/teensy31.html) The Teensy runs
the [MCard1802TeensyPixieVideo](https://github.com/fourstix/MCard1802TeensyPixieVideo) code to support a video ram buffer displayed on 
any 128 x 64 graphics display supported by the [U8G2 graphics library](https://github.com/olikraus/u8g2).  The Teensy 3.2 will
simulate the Interrupt Request, External Flag 1 (/EF1) signal, and DMA Output requests from the original pixie video.  This
allows [programs](https://github.com/fourstix/MCard1802ArduinoV2/blob/master/docs/Cdp1802SampleProgramCode.txt)
written for the original Cosmac Elf hardware to run directly on the simulator. The MCard1802 Teensy Pixie Video supports
32 x 64 and 64 x 64 bit video resolutions at the same speed as the cdp1861 Pixie Video chip.

U8G2 supports many kinds of 128 x 64 displays.  A list of supported displays is available 
[here.](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp)

For example, this [SSD1306 I2C 128 x64 OLED display](https://www.adafruit.com/product/938) available
from Adadruit works fine with the Qwiic interface and is supported by Uthe 8G2 graphics library.

This code uses the [MCP23017 Arduino library](https://github.com/blemasle/arduino-mcp23017) by Bertrand Lemasle
and the [Adafruit MCP23008 Arduino library](https://github.com/adafruit/Adafruit-MCP23008-library) to
communicate to the 1802 Membership card via I2C.

Serial Communication
--------------------
This program can be used to communicate with the Super Monitor program running on the MCSMP20J ROM supplied with
the Membership Card Starter Kit.  The serial communication uses inverse logic with the /EF3 line as RX and inverse Q as TX.
The code is based on the [MCard1802Terminal](https://github.com/fourstix/MCard1802Terminal) project.


This code uses the AltSoftSerial library by Paul Stoffregen for Serial communication to the 1802 Membership Card. 
This library is available through the Arduino Library Manager or in the [PaulStoffregen/AltSoftSerial repository](https://github.com/PaulStoffregen/AltSoftSerial)
on GitHub. For Inverse Logic an updated version of the AltSoftSerial library is available in the [fourstix/AltSoftSerial repository](https://github.com/fourstix/AltSoftSerial)
forked from the master repository on GitHub.  [Pull Request #59](https://github.com/PaulStoffregen/AltSoftSerial/pull/59) will merge this code update into the master.

Advantages
----------
* The second design decouples the Arduino from the 1802 Memory Card making it easier to replace the Arduino with
another processor.  Communication to the 1802 Membership card is soley through I2C and serial TX, RX lines.
* The LCD display provides more information than the Seven Segment display in the first design.
* The Front Panel Card provides a minimal support for the 1802 Membership Card using only 3 logic chips for Data I/O
and serial communication.
* The Daughter Card provides video, ROM and address display support.
* Serial communication is compatible with programs supplied in the MCSMP20J ROM.  It also supports other
serial terminal programs such as RealTerm and Putty.
* The Teensy 3.2 Pixie video is real-time compatible with code written for the CDP1861 chip and supports both 64x64
and 32x64 video resolution modes.

Teardown
--------
Teardown from the first version.

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Teardown1.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Teardown2.jpg"></td> 
  </tr>
  <tr align="center">
    <td>Removed all inside protoboards and logic, leaving only the Arduino.</td>
    <td>Replaced Seven Segment display with a generic 16x2 I2C LCD display and kept SH1106 128x64 OLED display.</td>
  </tr>
</table>

Front Panel
-----------
Front Panel card with MCP23008 for control lines, a 7400 Quad Nand logic chip for Memory Protect and inverse Q for serial communication TX line and MCP23017 IO Expander for data lines.

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/FP1_Bare.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/FP2_Populated.jpg"></td> 
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/FP3_Complete.jpg"></td> 
  </tr>
  <tr align="center">
    <td>Front panel card with sockets and connections for I2C, Vin, Ground and serial communication.</td>
    <td>Front panel card populated with MCP23008, MCP23017 and 7400 with 30 pin socket for 1802 Membership Card.</td>
    <td>Fully populated Front Panel card with 1802 Membership Card connected.</td>
  </tr>
  <tr align="center">
    <td colspan="3"><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/FrontPanelSchematic.jpg"></td>
  </tr>
  <tr align="center">
      <td colspan="3">Front Panel Card Schematic</td>
  </tr>
</table>

Daughter Card
-------------
Daughter Card for Pixie Video, Address display and ROM.  The Daughter Card plugs into socket U2 in the 1802 Membership Card.

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/DaughterCard.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/DC_Installed.jpg"></td> 
  </tr>
  <tr align="center">
    <td>Daughter card populated with Teensy 3.2, a 374 Data Latch, MCP23017 IO Expander and MCSMP20J ROM.</td>
    <td>Fully populated Daughter Card installed in the U2 socket of the 1802 Membership Card.</td>
  </tr>
  <tr align="center">
    <td colspan="2"><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/DaughterCardSchematic.jpg"></td>
  </tr>
  <tr align="center">
      <td colspan="2">Daughter Card Schematic</td>
  </tr>  
</table>

Assembly
--------
The Front Panel Card, Daughter Card and 1802 Membership Card were assembled in a painted cardboard box with switches
and displays.  The Serial Communication with MCSMP20J ROM programs were validated with the Arduino IDE and the ANSI
graphics with Adventureland program were verified with the RealTerm terminal program.

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Assembled_1.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Adventure.jpg"></td> 
  </tr>
  <tr align="center">
    <td>1802 Membership card with Hexadecimal Keypad, 16x2 LCD display and an SH1106 OLED display mounted in a cardboard box.</td>
    <td>RealTerm Window with the 1802 Membership Card running MCSMP20J ROM with Adventureland ANSI graphics.</td>
  </tr>   
</table>

Example Demos
-------------
Here are some examples running actual [CDP1802 programs.](https://github.com/fourstix/QwiicCosmacElfSim/blob/master/docs/Cdp1802SampleProgramCode.txt)


<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Demo1_Q.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Demo2_Spaceship.jpg"></td> 
  </tr>
  <tr align="center">
    <td>1802 Membership card with a Hexadecimal Qwiic rx4 Keypad, generic 16x2 LCD display and an SH1106 128x64 OLED display</td>
    <td>SH1106 128x64 OLED display with 1802 Membership card running Cosmac Elf Spaceship program.</td>
  </tr>
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Demo3_DMATest.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802ArduinoV2/blob/master/pics/Demo4_SecondsClock.jpg"></td> 
  </tr>
  <tr align="center">
    <td>1802 Membership card with SH1106 128x64 OLED display running the Video DMA Test program.</td>
    <td>1802 Membership card with SH1106 128x64 OLED display running the Digital Clock program..</td>
  </tr>  
</table>

Repository Contents
-------------------
* **/src/MCard1802v2FrontPanel/**
  * MCard1802v2FrontPanel.ino -- Arduino based Hex Keypad digital input with an LCD display used to show the data IO lines and status for the 1802 Membership card.
  * MCard1802Terminal.ino -- Serial terminal for communicating with 1802 Membership Card.
* **/src/MCard1802DaughterCard/** 
  * MCard1802DaughterCard.ino -- Arduino based LCD display used to show the address, data IO lines and status for the 1802 Membership card with
  Pixie Video on an 128 x 64 graphic OLED display. 
  * MCard1802Terminal.ino -- Serial terminal for communicating with 1802 Membership Card.
  * MCard1802v2Programs.ino -- Serveral classic 1802 programs with a program loader to load them into the 1802 RAM memory.  
* **/docs** -- documentation files
  * MCard1802v2FrontPanel.pdf -- schematic for Front Panel hardware with MCP23008 and MCP23017 circuit logic.
  * MCard1802v2DaughterCard -- schematic for Daughter Card hardware with Pixie Video simulation logic using an Teensy 3.2 and a MCP23017 IO Expander for Address
  lines and the MCSMP20J ROM.
  * Cdp1802SampleProgramCode.txt -- Sample 1802 code listings for various programs.
* **/pics** -- pictures of sample configurations and examples


License Information
-------------------

This code is public domain under the MIT License, but please buy me a beer
if you use this and we meet someday (Beerware).

References to any products, programs or services do not imply
that they will be available in all countries in which their respective owner operates.

Sparkfun, the Sparkfun logo, and other Sparkfun products and services are
trademarks of the Sparkfun Electronics Corporation, in the United States,
other countries or both. 

Adafruit, the Adafruit logo, and other Adafruit products and services are
trademarks of the Adafruit Industries, in the United States,other countries or both.

PJRC, the PJRC logo, and other PJRC products and services are
trademarks of the PJRC.com LLC, in the United States,other countries or both. 

Other company, product, or services names may be trademarks or services marks of others.

All libraries used in this code are copyright their respective authors.
  
Universal 8bit Graphics Library
Copyright (c) 2016, olikraus
All Rights Reserved
 
Sparkfun Qwiic Keypad Arduino Library
Copyright (c) 2016 SparkFun Electronics
Written by Pete Lewis @ SparkFun Electronics, 3/12/2019

Hex Keypad Arduino Library
Copyright (c) 2020 by Gaston Williams

Adafruit MCP23008 GPIO Expander Library
Copyright (c) 2012-2019 Adadruit Industries
Written by Limor Fried/Ladyada for Adafruit Industries.  

MCP23017 Arduino Library
Copyright (c) 2017 Bertrand Lemasle

Sparkfun SerLCD Arduino Library
Written by Gaston Williams and Nathan Seidle @ SparkFun
Copyright (c) 2018-2020 Sparkfun Electronics

LiquidCrystal_I2C Arduino Library
Written by Frank de Brabander
Copyright (c) 2011 DFRobot.com

Alternative Software Serial Library
Written by Paul Stoffregen
Copyright (c) 2014 PJRC.COM, LLC, 

The Teensy 3.2 hardware and software
Copyright (c) 2016-2020 by Paul Stoffregen, PJRC.com LLC 

The 1802 Membership Card Microcomputer 
Copyright (c) 2006-2020  by Lee A. Hart.
 
Many thanks to the original authors for making their designs and code avaialble as open source.
 
This code, firmware, and software is released under the [MIT License](http://opensource.org/licenses/MIT).

The MIT License (MIT)

Copyright (c) 2020 by Gaston Williams

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.**