
# Welcome to the project of modifying registers for TRINAMIC drivers, but now ANALOG DEVICES...

## About the project
The Klipper firmware provides powerful capabilities for controlling 3D printers, however, in order to achieve maximum performance and proper operation of such an important component as the “drive”, careful tuning of stepper motor control parameters is required.
Our project was created to provide you with guidance on optimizing driver parameters for various types of steppers.

### Currently the program supports the following kinematics: `CoreXY / Hbot / Cartesian` `2/4 WD`

## Introduction
Why is it needed?
 1. There are at least 2 "main" registers, these are none other than TBL (Comparator blank time) and TOFF (Turn-Off Time), they are both responsible for disconnecting and supplying power to the mosfets, which in turn open to the stepper motor windings, thereby affecting the loss of currents. Based on this small context, you can
    we can already conclude that by selecting the correct current control for our windings, we reduce interference, "extra" currents that interfere with the operation of the motor. That is, they knock down the angle of the rotor, vibrate, and the energy is simply transformed into heat, absorbed by the body, and this, in turn, means that the rotor torque is also lost.
 2. To summarize, of course stepper motors can be used on standard registers, but this is simply inefficient. Of course, it may happen that the standard registers will be optimal for your stepper, but this is unlikely.
    After conducting research, we found out that the correct selection of engine control modes gives it up to 30% torque, reduces vibrations up to 10 times. As for heat dissipation, it decreases by ~15% in my case. (This test is very lengthy, I don't have any other feedback).
 3. It is important to note that the registers are relevant only for a specific pair: `driver - motor`, and depend on the specific implementation of the driver. It is also worth considering that the characteristics of the motor, according to the manufacturer, have up to +-20% spread, as well as the measurement error, accelerometer, so you should not be surprised at the run-up of the parameters on different installations.

## Assistance
We welcome contributions from the community! If you have experience and advice on tuning stepper motors, do not hesitate to share the material with us. Provide information about the tested stepper motors.

### Further [instructions](/wiki/wiki.md), good luck!

### [Support](https://ko-fi.com/altzbox) project

### Contacts - @altzbox @mrx8024 telegram / discord
