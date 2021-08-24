# Code Readme

i2c_accel.c
-----------
This program handles all of our sensor data, the udp client task, and recieving packets for the button press. 

Starting on line 159 is where we concatenate all of our sensor data. After this we send our data to the server to be recieved at hurricane.js
Right after this is where we handle the recieved packets for the button press from the site. From lines 191 to 221 we use the strncmp function 
to check if we have recieved the message Ok! or Nk!. If we recieve Ok! we toggle the LED, otherwise it stays off.

Lines 292 to 351 is the same program from wifi station given to us on the course site. This code sets up our esp32 as a wifi module to allow us to 
send and recieve data. 

Line 423 to 558 is the same program from the accelerometer skill. This is where we print out our data to console so that we are able to parse it and app.get in
hurricane.js. The specific print lines of the data are on lines 539-540, and 551-553.

Lines 621 to 685 is the same program from the thermistor skill. This utilizes the adc pin and converts the voltage to temperature units. We print out this data on lines 677 and 679 allowing it to be accessed by hurrican.js. 

Finally, in app_main we call all three tasks, accelhandler (for accelerometer data), thermoHandler (for thermistor data), and the udp_client_task. 

hurricane.js
------------
Server side, This program logs all of our sensor data and sends the data to our client at port 1130. Additionally, we also use the server.send to send packets to the c program to toggle the button press. 

Lines 56-106 is where we handle the button press and send a packet with the msg = "Ok!" or "Nk!" depending on the satus of the button when we app.get /button on line 116.

Lines 127 -147 is when we send all of our data to the client. 

client.js
------------
Client side, This program is the client side where we add our event listener and handle the button press. 

Specifically lines 9-11 handle the button press where we fetch and then listen so that the user can re-toggle the button again to turn off and on the LED. 

index.hbs
------------
This is the html program that sets up our webpage with all data headers and tags. Additionally, we use an image tag to display the motion webcam stream from the raspeberry pi. 

Finally, on lines 54-122 are all of our ajax calls to fetch data. The ajaz call is an asynchronousrequest initiated by the browser that does not directly result in a page transition, so this allows our data to updated dynamically. Specifically we do this by setting async to false. 









Also
- Please provide your name and date in any code submitted
- Indicate attributrion for any code you have adopted from elsewhere
