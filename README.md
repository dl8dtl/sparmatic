Sparmatic Comet / Zero V2 Firmware rewrite
==========================================

What is this all about?
-----------------------

This is a rewritten firmware for the "Sparmatic Comet" and
"Sparmatic Zero V2" electronic heating thermostats.  They have
been marketed cheap in some German supermarket chains years
ago.  They are based on an ATmega169 controller, a custom
LCD glass, and a motor to operate the valve.  A simple menu
system, arranged around three buttons and a thumbwheel,
complete the system.

Details
-------

There is a lengthy thread in the (German) mikrocontroller.net
forum dealing with all the details of these devices:

https://www.mikrocontroller.net/topic/237375)

https://www.mikrocontroller.net/topic/237375?page=1#2407844
contains links to schematics and photos of the device.

Wiki page with many details (in German language):

https://www.mikrocontroller.net/articles/Sparmatic_Heizungsthermostate

Firmware
--------

This project is a complete firmware rewrite.  Knut Ballhause provided
assembly code rewrites which are licensed as Public Domain (CC0), and
are available in the directories Sparmatic_Comet_M169_AVR5/ and
Sparmatic_V3_Thermostat_M169PA_ATM6/.

The subdirectory C/ contains a rewrite of Knut's assembly code in C.
Originally, the goal of the rewrite was to produce a bug-to-bug
compatible C counterpart for the assembly code, and it has been worked
on from that. This code is by Jörg Wunsch, and is also licensed as
Public Domain.
