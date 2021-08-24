# Code Readme

This code is adopted from the timer example from the code examples github provided to us.
We chose to use the timer example code as a launching point because it allowed us to easily utilize the ESP32's 
built-in internal timer functionality.

Building off of the timer example code, we wanted to create two new tasks to handle the servo operation and the alphanumeric display functionality, in addition to modifying the existing timer task. 

To begin, we modified the timer_evt_task() by removing the previous actions and replacing them with an action that decrements our global counter variable on each clock cycle. We do this using the evt.flag conditional on line 367. See our code for comments.

Next, we built the displayHandler() task as a way to visually display the current state of the timer. We did this by reusing code from our Skill 8 assignment, initializing all of the necessary functions for the I2C alphanumeric display. After this, we used the modulo and integer division operators to break down the counter variable from seconds into minutes and seconds. We then update the display by indexing the alphafonttable struct that we declared globally. This struct contains the binary equivalents of integers 0-9. This task is commented and can be found on line 426.

Next, we built the servoHandler() task in order to control the servo motion at the end of the timer countdown. This task was largely able to be recycled from our Skill 9 code.  Once the timer finishes counting down, we first reset the counter variable to 120 seconds, so that it can count down without loss while the servo performs its actions. After resetting the counter variable, we use the mcpm_set_duty_in_us() function to specify an exact angle for the servo to move to. We tell the servo to move between 20 and 80 degrees rotation, as this range gave us the smoothest operation. This task is commented and can be found on line 377.

