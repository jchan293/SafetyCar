Safety Car
by Pranav Peddinti and Jorlly Chang.

The safety car project was our final project for the ME 507 course at Cal Poly. It involves modifying an existing remote controlled car, 
incorporating additional hardware, including an ESP32, time of flight (ToF) sensor, motor driver, and accelerometer to allow the car to obtain 
real world automotive safety features such as automatic emergency braking (AEB) and an antilock braking system (ABS).

This repository consists of the software portion of the project. The code allows the vehicle to obtain data from the ToF sensor and utilize this 
data to control a motor driver to stop the car before it collides with an object. The ToF sensor measures the distance in front of the car while 
it is moving forward, and if it detects an object one meter away or less, it will immediately cut power to the motor. A future iteration of the 
code will keep the motor activated but with no forward motion, allowing the car to stop immediately rather than coast to a stop.

The motion of the car can be changed using a webpage that is hosted on the ESP32, and can be connected to over Wi-Fi. This control allows the user 
to switch the direction of movement, like a gear selector, with Drive, Reverse, and Park. Each one of these buttons is linked to a function in the 
code that tells the car to go forward, backwards, or stay in place.
 
 This project is intended to replicate and observe how important safety features can be incorporated into a much smaller scale. These lifesaving 
technologies are essential for people on the road, both drivers and pedestrians. While this iteration of the code worked as intended, reducing the 
risk of a severe impact, there is still room for improvement to completely eliminate the risk of a collision altogether.
