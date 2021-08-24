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
let button1 = 0;
let button2 = 0;

// Global variables for sensor data
let buttonOn = 0;
let buttonOff = 0;


// Port and IP
var PORT = 9001;
var HOST = "192.168.1.164";

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
  if (button1 == 1) {
    button1 = 0;
    server.send("On!", remote.port, remote.address, function (error) {
      if (error) {
        console.log("Did not send signal");
      } else {
        console.log("Sent: On!");
      }
    });
  }
  else{
    server.send("No!", remote.port, remote.address, function (error) {
      if (error) {
        console.log("Did not send signal");
      } else {
        console.log("Sent: On!");
      }
    });
  
  }
  // if (button2 == 1) {
  //   button2 = 0;
  //   server.send("Of!", remote.port, remote.address, function (error) {
  //     if (error) {
  //       console.log("Did not send signal");
  //     } else {
  //       console.log("Sent: Of!");
  //     }
  //   });
  // }
});

// Bind server to port and IP
server.bind(PORT, HOST);

//serves the html file on the client side
app.get("/", function (req, res) {
  res.render("index.hbs");
});

app.get("/button", (req, res) => {
  //change button press variable to true
  console.log("BUTTON 'ON/OFF' PRESSED");
  button1 = 1;
  res.send({
    message: "TOGGLE"
  });
});

// app.get("/button2", (req, res) => {
//   //change button press variable to true
//   console.log("BUTTON 'OFF' PRESSED");
//   button2 = 1;
//   res.send({
//     message: "OFF"
//   });
// });

//Serve on localhost:8080
app.listen(1130);