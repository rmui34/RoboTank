# RoboTank
Design Specification

In this project we are required to develop a functional RoboTank system and controller that operates via Bluetooth Serial Port Profile (Bluetooth SPP). The RoboTank system is a Real-Time Operating System that includes the implementation of GPIO peripherals for a keypad, OLED user-interface, ADC sensors, alarm system, two pulse width modulators (PWMs), and an UART port for Bluetooth Communication. Both the RoboTank and controller run using interrupt and task driven functions. All data must be safely shared and protected using semaphores. Protecting the shared data will prevent critical failure of the RoboTank system.
The final product will satisfy these basic requirements and additional features:
A user-interface that is controlled by the keypad. The menu will allow the user to access sub-menus and choose between different modes. This interface will display information about the current state of the RoboTank on the controller OLED.
Use instantaneous and average ADC sensor data to display the distance. This data will be used to help the RoboTank system avoid objects.
Generating two PWMs to control the motors of the RoboTank System.
A set of motor control commands that will adjust the PWMs to drive the RoboTank forward, reverse,left, right, forward-left, forward-right, reverse-left, reverse-right, clockwise rotation, and counter-clockwise rotation.
The controller and RoboTank will communicate through Bluetooth.
An auto-stop feature that stops the Robo-Tank when an object is too close to the front sensor.
An alarm system that increases in frequency when the back sensor sees an object while reversing.
A semi autonomous and autonomous mode giving the ADC sensors some and full control of the RoboTank system.
RoboTank Inputs: ADC sensor data and receive control commands via UART using Bluetooth SPP.
RoboTank Outputs: PWMs the control the motor and the operation of RoboTank.
Controller Inputs: User key presses to select modes, and control commands.
Controller Outputs: Control commands sent out via UART using Bluetooth SPP.
