# 2-phase_90-_VFD
Atmel 90PWM3B code for controlling speed of capacitor asynchronous motor

This is a code for Atmel 90PWM3B for speed control of capacitor asynchronous motor by Variable Frequency Device (VFD). I built such a device to control the condenser fan of my self-made chiller. The program and the circuit can be used for any capacitor asynchronous motor. The hardware is described here: http://gigavolt.eu/projects/chillerR290/chillerR290.html

The program as well as the circuit diagram is offered as it is and this repository is not maintained. I offer it as an aid for experienced persons and refuse any kind of liability.

The motor frequency is set by duty cycle of the control PWM signal. A half of sinus wave with amplitude of 255 must be stored in the 512 bytes long EEPROM. Two samples with 90° phase shift are read periodically and sent to corresponding half-bridge PWM stages of the Atmel microcontroller. The amplitude of both sine waves is set inverse-proportional to the frequency to avoid saturation of the motor magnetic circuit. 100% amplitude is set at 50Hz frequency.
