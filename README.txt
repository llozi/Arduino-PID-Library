This is a fork of Arduino PID Library - Version 1.2.1, https://github.com/br3ttb/Arduino-PID-Library

It changes all of the double calculations to float data type as single precision would be sufficient.
When used with Arm M4 based controllers double would be 64bit. On AVRs this doesn't matter as float
and double are the same there.

floats are used if the library is compiled with the macro USE_FLOAT defined, for this you may hand over
a -D USE_FLOAT to the compiler, otherwise still doubles are being used.

Changes Copyright 2022, Lukas Zimmermann, Basel, Switzerland

Original:
***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

 - For function documentation see:  http://playground.arduino.cc/Code/PIDLibrary
