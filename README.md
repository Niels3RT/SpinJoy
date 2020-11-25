# SpinJoy - Spinner and Joystick combo

## Introduction
It's a spinner/joystick combo device based on [Sorgeligs PaddleTwoControllersUSB project](https://github.com/MiSTer-devel/Retro-Controllers-USB-MiSTer/tree/master/PaddleTwoControllersUSB) in a single Arduino Pro Micro (ATmega32U4) board.

The Arduino shows up as 2 devices, a digital joystick+12 buttons with added wheel+dial and a mouse that moves the x/y axes by rotating the spinner. This works nicely with Mister and was tested on Arkanoid and Block Gal cores.

![SpinJoy](SpinJoy.png?raw=true "SpinJoy")

### Spinner
You can use almost any 2-phase spinner. Hight-PPR (300+) spinners provide more smooth scroll while even simple miniature Arduino shield spinner with 20PPR is fine.
I used a WISAMIC 600p/r Incremental Rotary Encoder DC5-24v as cheaply sold on EBay, AliExpress or Amazon.

### Joystick/Buttons
There are many Arcade Joystick/Button sets available on EBay, AliExpress or Amazon.
I bought the cheapest, any other should work too.

### Connecting/Assembling

![Arduino Pro Micro Pinout](Pinout.png?raw=true "Arduino Pro Micro Pinout")

Rotary encoder:
| Wire Color | Function |
|------|--------|
| red | 5v |
| black | gnd |
| white | Enc A |
| green | Enc B |

The ribbon cable for the joystick fits nicely to pin headers soldered to the Arduino, the cables to the buttons fit pin headers too:
![Arduino Joystick connector](JoystickConnect.png?raw=true "Arduino Joystick connector")

### Source parameters
Firmware has several definitions to tweak for best experience:

1. SPINNER_PPR: set the correct PPR value according to your spinner for correct work.
2. DEBOUNCE_TOP: a counter is used to debounce buttons and stick movement. Set to 1 to completely disable debouncing. The switches in my (cheeaaap) joystick/button set need a value of 32 to stop bouncing, your mileage may vary. The menus in Mister scripts seem to be a good place to test this. This counter is only applied to switch release to minimize lag.
3. USE_AUTOFIRE: comment to disable autofire function, buttons 11/12 are reported via USB in any case.
4. AUTOFIRE_TUNE_FACTOR: bigger number -> faster autofire rate modification, lower number -> finer adjustment, default is 4

### Autofire
Autofire for button 1 is enabled by pushing button 12 (as long as its down), if you plan to use it its a good idea to connect a switch there.

To modify the fire rate hold down button 11 and use the spinner. The new rate is saved to eeprom when button 11 is released.

### Drill Template
There is a simple drill template included for mounting the rotary encoder and the joystick. Please verify the dimensions of the printed PDF before drilling, its for A4 paper size and scaling may be involved in printing.
A DXF file is also included to move things around in LibreCAD.

## License
This project is licensed under the GNU General Public License v3.0.
