Overview
========
The Wakeword low power project is a demonstration that presents how to deploy low power local voice on cortex M. It performs the recording of the microphone connected to the SAI 5 of the board and the Keyword detection. This solution integrates the Voicespot™ wake word engine from our partner RetuneDSP. This application has the purpose of showing an example of integration code. Indeed, the Voicespot library isn't linked to this application so the user won't be able to run this application as it is. But the user will be able to use this as a template to integrate either Retune solution or another one. Once the user will have integrated its Keyword engine, he/she will be able to run it.

The example application creates one task called wake_word_low_power_demo. This task print "Hello world." message
via debug console utility and suspend itself.

For more information, read the application note "i.MX 8M Mini Heterogenous Low Power Voice Control Solution" available on nxp.com


SDK Version
===================
MCUxpresso SDK_2.8.0

Toolchain supported
===================
- GCC ARM Embedded:  gcc-arm-none-eabi-8-2018-q4-major

Hardware requirements
=====================
- Micro USB cable
- MIMX8MM6-EVK  board
- J-Link Debug Probe
- 12V power supply
- Personal Computer
- Ribbon, 4 female-female wires and 60 pins connector to connect mic to board
- 1 Omnidirectional Microphone with I2S Digital Output. We use https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf
- Optional: headphones (usedto test recording on M4-everything recorded by the mic will be played to the headphones).

Board settings
==============
No special settings are required.



Prepare the Demo
================
1.  Connect 12V power supply and J-Link Debug Probe to the board, switch SW101 to power on the board
2.  Connect a USB cable between the host PC and the J901 USB port on the target board.
3.  Connect the headphone and the microphone on the target board. 
4.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
5.  Download the program to the target board.
6.  Launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the demo runs successfully, you can hear the recording and the keyword is detected.

