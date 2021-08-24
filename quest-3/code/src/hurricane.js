var fs = require("fs");
const path = require("path");
const express = require("express");
var dgram = require("dgram");
const hbs = require("hbs");

const app = express(); //doesn't take in args

var payload = "";
var data = []; // Array to store payload data from UDP datagram

//Define paths
const publicDirPath = path.join(__dirname, "../public");
//this is optional (allows you to call views directory by another name) Make sure you use app.set('views',viewsPath)
const viewsPath = path.join(__dirname, "../public");

//express needs the first key to be view engine and second to be hbs
//this sets up handlebars
app.set("view engine", "hbs");
app.set("views", viewsPath);
app.use(express.static(publicDirPath));

// Global variable to keep track of LED Button status
let ledButtonPress = 0;

// Global variables for sensor data
let thermistorData = 0;
let accelXData = 0;
let accelYData = 0;
let accelZData = 0;
let accelRollData = 0;
let accelPitchData = 0;

// Port and IP
var PORT = 9001;
var HOST = "192.168.1.9";

// Create socket
var server = dgram.createSocket("udp4");

// Create server
server.on("listening", function () {
  var address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});

// On connection, print out received message
server.on("message", function (message, remote) {
  console.log(remote.address + ":" + remote.port + " - " + message);
  payload = message.toString();
  console.log("Payload: ", payload);

  // Send Ok acknowledgement
  if (ledButtonPress == 1) {
    ledButtonPress = 0;
    server.send("Ok!", remote.port, remote.address, function (error) {
      if (error) {
        console.log("MEH!");
      } else {
        console.log("Sent: Ok!");
        data = payload.split(",");
        console.log(data);

        thermistorData = data[5];
        accelXData = data[0];
        accelYData = data[1];
        accelZData = data[2];
        accelRollData = data[3];
        accelPitchData = data[4];

        console.log(thermistorData);
        console.log(accelXData);
        console.log(accelYData);
        console.log(accelZData);
        console.log(accelRollData);
        console.log(accelPitchData);
      }
    });
  } else if (ledButtonPress == 0) {
    server.send("Nk!", remote.port, remote.address, function (error) {
      if (error) {
        console.log("MEH!");
      } else {
        console.log("Sent: Nk!");
        data = payload.split(",");
        console.log(data);

        thermistorData = data[5];
        accelXData = data[0];
        accelYData = data[1];
        accelZData = data[2];
        accelRollData = data[3];
        accelPitchData = data[4];

        console.log(thermistorData);
        console.log(accelXData);
        console.log(accelYData);
        console.log(accelZData);
        console.log(accelRollData);
        console.log(accelPitchData);
      }
    });
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);

//serves the html file on the client side
app.get("/", function (req, res) {
  res.render("index.hbs");
});

app.get("/button", (req, res) => {
  //change button press variable to true
  console.log("BUTTON PRESSED");
  ledButtonPress = 1;
  res.send({
    message:"YEET"
  });
});

// Data Routes

//Send thermistorData
app.get("/data1", function (req, res) {
  res.send(thermistorData);
});
//Send accelXData
app.get("/data2", function (req, res) {
  res.send(accelXData);
});
//Send accelYData
app.get("/data3", function (req, res) {
  res.send(accelYData);
}); //Send accelZData
app.get("/data4", function (req, res) {
  res.send(accelZData);
}); //Send accelRollData
app.get("/data5", function (req, res) {
  res.send(accelRollData);
}); //Send accelPitchData
app.get("/data6", function (req, res) {
  res.send(accelPitchData);
});

//Serve on localhost:8080
app.listen(1130);
