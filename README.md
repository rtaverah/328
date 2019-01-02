
ArduEMS-328
===========

Arduino Engine Management System for Arduino boards with the ATmega328p.
The supported board are:
 - Arduino Uno
 - Arduino Duemilanove
 - Arduino Diecimila
 - Arduino NG
 - Arduino Extreme v2
 - Arduino Extreme

Uploading the Firmware
======================

Either use [AVRDude](http://www.nongnu.org/avrdude/) or [XLoader](http://xloader.russemotto.com/) to upload the .hex file.


Compiling from Source
=====================

Use the supplied Makefile to compile and upload. On linux, you will need
all of the AVR-Libc toolchain. On windows you will need WinAVR.

You will of course also need a Make utility for your specific operating system.

The commands are:
  make all		(compile the whole project)
  make program		(upload hex file to the arduino board)
  make clean		(clean out built project files)
  
You will have to set your usb port in the Makefile. Default port is COM4.
Look through Makefile for more options.

Tuning
======

Use MegaTune, TunerStudio or MegaTunix. Run as a Megasquirt 1 B&G2.0-3.0
