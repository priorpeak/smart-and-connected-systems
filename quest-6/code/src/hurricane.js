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
let catButton = 0;
let dogButton = 0;
let Nk = 0;

// Global variables for sensor data
let thermistorData = 0;
let lidarData = 0;
let ultrasonicData = 0;
let dogCounter = 0;
let catCounter = 0;
let dogMotion = false;
let catMotion = false;

const dogFeedDistance = 30;
const catFeedDistance = 50;
// let accelZData = 0;
// let accelRollData = 0;
// let accelPitchData = 0;

// Port and IP
var PORT = 9001;
var HOST = "192.168.1.153";

// Create socket
var server = dgram.createSocket("udp4");

// Create server
server.on("listening", function () {
  var address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});

//lastLidar = 0;
//lastUltrasonic = [0, 0, 0, 0, 0];

// On connection, print out received message
server.on("message", function (message, remote) {
  console.log(remote.address + ":" + remote.port + " - " + message);
  payload = message.toString();
  console.log("Payload: ", payload);

  data = payload.split(",");
  console.log(data);

  // If received payload is from sensor ESP:
  if (data[0] == "0") {
    thermistorData = data[1];
    lidarData = parseInt(data[2]);
    ultrasonicData = parseInt(data[3]);

    console.log(thermistorData);
    console.log(lidarData);
    console.log(ultrasonicData);
    if (lidarData <= dogFeedDistance) {
      dogCounter++;
      console.log("INCREMENTING DOG_______________ \n");
    }
    if (ultrasonicData <= catFeedDistance) {
      catCounter++;
      console.log("INCREMENTING CAT_______________ \n");
    }
    if (dogCounter == 5) {
      console.log("DOG TRIPPED");
      dogMotion = true;
      dogCounter = 0;
    }
    if (catCounter == 5) {
      console.log("CAT TRIPPED");
      catMotion = true;
      catCounter = 0;
    }
    //dogButton = 0;
    server.send("Nk!", remote.port, remote.address, function (error) {
      if (error) {
        console.log("MEH!");
      } else {
        console.log("Sent: Nk!");
      }
    });

    // server.send("Nk!", remote.port, remote.address, function (error) {
    //   if (error) {
    //     console.log("MEH!");
    //   } else {
    //     console.log("Sent: Nk!");
    //   }
    // });
  } else {
    // Send Ok acknowledgement
    if (catButton == 1 || catMotion == true) {
      catButton = 0;
      catMotion = false;
      server.send("Cat!", remote.port, remote.address, function (error) {
        if (error) {
          console.log("MEH!");
        } else {
          console.log("Sent: Cat!");
        }
      });
    } else if (dogButton == 1 || dogMotion == true) {
      dogButton = 0;
      dogMotion = false;
      server.send("Dog!", remote.port, remote.address, function (error) {
        if (error) {
          console.log("MEH!");
        } else {
          console.log("Sent: Dog!");
        }
      });
    } else {
      //dogButton = 0;
      server.send("Nk!", remote.port, remote.address, function (error) {
        if (error) {
          console.log("MEH!");
        } else {
          console.log("Sent: Nk!");
        }
      });
    }
  }
});

// Bind server to port and IP
server.bind(PORT, HOST);

//serves the html file on the client side
app.get("/", function (req, res) {
  res.render("index.hbs");
});

app.get("/catButton", (req, res) => {
  //change button press variable to true
  console.log("CAT BUTTON PRESSED");
  catButton = 1;
  res.send({
    message: "YEETING SOME CAT FOOD",
  });
});

app.get("/dogButton", (req, res) => {
  //change button press variable to true
  console.log("DOG BUTTON PRESSED");
  dogButton = 1;
  res.send({
    message: "YEETING SOME DOG FOOD",
  });
});

// Data Routes

//Send thermistorData
app.get("/data1", function (req, res) {
  res.send({ data: thermistorData });
});

app.get("/data2", function (req, res) {
  res.send({ data: lidarData });
});

app.get("/data3", function (req, res) {
  res.send({ data: ultrasonicData });
});

app.get("/data4", function (req, res) {
  res.send({ data: dogMotion });
});

app.get("/data5", function (req, res) {
  res.send({ data: catMotion });
});

//Serve on localhost:8080
app.listen(1130);
