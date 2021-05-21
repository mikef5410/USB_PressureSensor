## USB_Pressure_Sensor

USB measurement of Air pressure (compressor monitoring)


This project is an STM32F373 based USB gadget that measures air pressure in order to monitor compressor state.

The hardware design files are in doc/

This device uses FreeRTOS, and implements two interfaces on the bus...

1. A CDC-ACM interface with a simple commandline monitor
2. A simple COMMAND-RESPONSE packet interface on bulk endpoints 0x1/0x81. It's
this interface that the perl code in perl/ and perl/test talks to. It's made to
be machine-machine.

The controller consults EEprom for USB identifying material, so one board and
software load can fit many uses.

After cloning, run the bootstrap script to get the required submodules.

