# Code Readme

## udp_client.c
This is our main c program for the quest where we control all sensor readings, drive and steering control, and handling and receiving udp messages.

Lines 81-156 is our udp_client_task. Here we constantly send a message to our node.js server. Once the message is handled, the server sends the message 'On!' back. 
The most important part of this task is on lines 132-138. This is where we check to see if we received 'On!'. If we have, we toggle our boolean variable, 'startCrawler' which is used to start and stop the crawler in the PID task starting on line 652.

Lines 232 - 411 are multiple I2C functions used to define and setup I2C buses. In our program we use a first I2C bus for the ultrasonic sensor and a second I2C bus for the alpha numeric display. 

Lines 413 - 449 is our lidarRead function. This takes the 9 byte hex output from the microlidar and converts it into a single 16 bit value which is our distance from the wall in centimeters.

Lines 498 - 562 control our alpha numeric display. This display is updated using the wheel speed received from PID. To achieve this we defined an alphaFontTable on line 215 with all digits and a comma to seperate the ones and tens place from the tenths and hundredths. We then update the last two segments of the display with the wheel speed. 

Lines 568 - 619 consits of our pwm initialization and calibration.

Lines 621 - 632 is extremely important in our program and is our calibrateESC() function. This is essential to driving the car and sets the high, low, neutral, and reset signals for the driving servo. 

Lines 652 - 693 is our PID speed control function for collision detection. Using integral, derivative, and previous error variable we are able to use an algorithm seen on lines 682 - 685 to determine the output that will be used for our wheel speed. However, this output used to control wheel speed is only updated once the ultrasonic sensor reads 20cm or less. These conditionals can be seen inside the function on lines 669 and 675.

Lines 734 - 778 is our steering control task. This task uses the lidarRead function's output to determine how close we are to the wall. Using conditionals on lines 757 and 765 we determine if we are too close or too far and adjust the steering servo angle accordingly to stay on course. 

Line 780 - 830 are our speedCalc function and opticalData function. These go hand in hand to determine wheel speed. Using the optical sensor we increase a counter each time the voltage increases on line 820. This corresponds to how many dark segements we have seen on our wheel. Using this information, the circumference on the wheel and a sampling period, we determine the wheel speed on line 787. 

Finally, Lines  832 - 860 output our ultrasonic data into variable range on line 856. This range is used constantly in the PID task to avoid collisions. 


## crawler.js
This is our node.js prorgam which handles receiving udp message from the c program and on receiving responding with a message on a button click from our server. 

Lines 48-84 control receiving the message from the client and deciding how to respond. Using a conditional on line 54 where if the button has been clicked, we then send the 'On!' message to the client allowing it to start/stop the car. Otherwise, we send 'No!'.

Lines 94 - 101 get the state of the button and when it is clicked we set the status of the variable 'button1' to 1 which is used in the conditional on line 54 to send the message. 

## client.js
This is our client side program that sets up the button event and allows us to check that we have pressed the button from the server. 

Most importantly, lines 7-17 add the button event listener. 

## index.hbs
This is our front side program using the client.js script and setting up the webpage with a single toggle button. 

Line 15 uses a button id to constantly check when the button is pressed which is fetched from the client.js program.
