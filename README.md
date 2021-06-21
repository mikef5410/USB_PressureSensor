## USB_Pressure_Sensor

USB measurement of Air pressure (compressor monitoring) The device also
includes an ambient air temperature, pressure, humidity monitor. As it happens
some of my work depends on a functioning air compressor, and ours is,
er... less than reliable. So, this device allows some of my automation to keep
an eye on the compressed air state, as well as air temperature, humidity and
pressure in the lab.

![USB Pressure Sensor as built](/doc/PressureSensor_asbuilt.jpg)

![USB Pressure Sensor as installed](/doc/PressureSensor_installed.jpg)

This project is an STM32F373 based USB gadget that measures air pressure in order to monitor compressor state.

The hardware design files are in Hardware, and a 3D-printable case is in Hardware/Mechanical.

This device uses FreeRTOS, and implements two interfaces on the bus...

1. A CDC-ACM interface with a simple commandline monitor
2. A simple COMMAND-RESPONSE packet interface on bulk endpoints 0x1/0x81. It's
this interface that the perl code in perl/ and perl/test talks to. It's made to
be dead simple machine-machine.

The controller consults EEprom for USB identifying material, so one board and
software load can fit many uses. perl/test/AttenSwitchTool can be used to modify the eeprom.

After cloning, run the bootstrap script to get the required submodules.

The 32F373 unfortunately needs an external 1500Ω pullup on USP_P to 3.3V which needs to be butched onto the board. It may be better to butch this to a GPIO output, but for now my board is using +3.3V.

