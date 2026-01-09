# Tape printer

Reinventing the printer - writing on a roll of tape with a sharpie.

## Overview
This project uses `GRBL` to control a stepper motor and two RC servos in a
CNC-machine-with-rotary-axis style. An `Atmega8` MCU is used to map the step/dir
control pulses from `GRBL` into servo commands.


## Printing
This project provides python code to generate GCODE for printing provided 
text string and font name+size.

Run the code in `print.ipynb` notebook:

```console
cd printing_wizard
jupyter notebook
```

The result of the above is a `text.nc` file that can be used in your
GCODE sender app of choice.


## Build

### Flashing `GRBL`
Check out the latest version of `GRBL`.
Use the config file provided in `grbl/config.h` to replace/merge with the `GRBL` one.
Compile and flash on an Arduino Nano.


### Flashing the `stepper-to-servo` firmware
Open the `stepper-to-servo` Arduino project and compile.
Copy the path to the compiled `stepper-to-servo.ino.hex` from the compiler log.

Attach your programmer to the Atmega8 and from the `avrdude` folder, run:
```console
# Set the fuses for 8MHz internal oscillator
.\avrdude.exe -v -patmega8 -cusbasp -Pusb -U lfuse:w:0xd4:m -U hfuse:w:0xd9:m

# Flash the firmware
.\avrdude.exe -v -patmega8 -cusbasp -Pusb -Uflash:w:PATH_TO_arduino_build/stepper-to-servo.ino.hex:i
```

### Schematic / wiring

![Schematic](./schematic.png)