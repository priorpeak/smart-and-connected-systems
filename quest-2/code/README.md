# Code Readme

The code for this skill was modified from our Quest 1 timer code. We made this decision because it had the RTOS Task framework laid out, and we wanted to make each sensor module an individual task. By doing this, we were simply able to paste our code from Skills 13, 14, and 15 into three unique tasks, found on lines 101, 161, and 211. Inside app_main() on line 261, we use xTaskCreate and set priorities to the three sensor tasks.

The majority of our time was spent working on the serial.js and canvasJS.html files. This is where we interface with our ESP32 using the serial connection. In serial.js, we use the serialport module to interface with the ESP32. We send data from the ESP32 in a CSV format, and sort this data into arrays using the .split() method. This way, we are able to read incoming data and properly sort between thermistor data, IR rangefinder data, and ultrasonic rangefinder data. After sorting the relevant data into their respective arrays, we set up three express routes to handle the data transfer from backend to frontend. These express routes can be found starting on line 40. 

In canvasJS.html, we initialize three global variables to store the arrays that are passed from the backend. Upon loading, we use a JQuery ajax call to repeatedly receive incoming data from the ESP32, and update the global arrays with the ten most recent sensor readings. Then, we populate the canvasJS chart object with the correct data. This happens on line 86 down. We also display the sensor data below the chart on the HTML page using document.getElementById. This can be found on line 42 down. 