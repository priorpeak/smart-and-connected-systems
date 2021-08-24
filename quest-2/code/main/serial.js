const SerialPort = require("serialport");
var fs = require("fs");
const path = require("path");
var app = require("express")();

const port = new SerialPort("/dev/cu.SLAB_USBtoUART", {
  baudRate: 115200,
});

let data;
let thermistorData = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
let irData = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
let ultrasonicData = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

//read data from ESP port
port.on("readable", function () {
  let data = String(port.read()).split(",");
  //the data is sent as "Temperature,20" so we split it by the comma and populate the appropriate array
  if (data[0] == "Temperature") {
    thermistorData.shift();
    thermistorData.push(Number(data[1].slice(0, -2)));
  } else if (data[0] == "irDistance") {
    irData.shift();
    irData.push(Number(data[1].slice(0, -2)));
  } else if (data[0] == "ultraDistance") {
    ultrasonicData.shift();
    ultrasonicData.push(Number(data[1].slice(0, -2)));
  }

});

function toggleDataSeries(e) {
  if (typeof e.dataSeries.visible === "undefined" || e.dataSeries.visible) {
    e.dataSeries.visible = false;
  } else {
    e.dataSeries.visible = true;
  }
}

//serves the html file on the client side
app.get("/", function (req, res) {
  res.sendFile(__dirname + "/canvasJS.html");
});

//Sends the thermistorData array
app.get("/data1", function (req, res) {
  res.send(thermistorData); // Send array of data back to requestor
});
//Sends the irData array
app.get("/data2", function (req, res) {
  res.send(irData); // Send array of data back to requestor

});
//Sends the ultrasonicData array
app.get("/data3", function (req, res) {
  res.send(ultrasonicData); // Send array of data back to requestor

});

//Serve on localhost:8080
app.listen(8080);
