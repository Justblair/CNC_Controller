# CNC_Controller
An Arduino based motor controller for a cheap DC Spindle.  

This is a work in progress.  The controller is designed to convert a cheap DC motor controller (555 timer based) that is found on yooCNC controllers with a smarter brain.

The controller uses optical measurement of the spindle speed plus a PID library to maintain motor speed despite the material

It also uses a current sensor to monitor the current through the MOSFET and motor... Should there be a spindle jam it will cut power to the spindle and send a kill command to the computer (via either a simulated keyboard stroke or by setting a pin low for the stepper motor controller to detect)

Working so far:
* Able to set speed via a variable resistor
* Graphic display provides user interface
* Current sensing, with ignore command when spindle speed is increased (to prevent power surges creating panic)
* PID in free air

To Do:
* Add PWM control from the PC (Via the stepper motor controller and Parrallel port)
* Add switch to set Manual/PC control of PWM
* Possible temperature monitoring of spindle and or mosfet?
* Possible Audio feedback
* Much better documentation of the project!
