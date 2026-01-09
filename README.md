# Tape printer



## Flashing the `stepper-to-servo` firmware
open the `stepper-to-servo` arduino project and compile.
Get the path to the compiled `stepper-to-servo.ino.hex` from the compile log.

Attach your programmer to the Atmega8 and from the `avrdude` folder, run:
```cmd
.\avrdude.exe -v -patmega8 -cusbasp -Pusb -Uflash:w:PATH_TO_arduino_build/stepper-to-servo.ino.hex:i
```