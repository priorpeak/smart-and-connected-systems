//John Kircher, Alex Prior, Allen Zou
//this is the node.js program to be run on the pi

var dgram = require("dgram");
var fs = require("fs");
const readLine = require('readLine');
var http = require('http');

const path = require("path");
const express = require("express");

const hbs = require("hbs");

const app = express(); //doesn't take in args

//app.use(express.urlencoded());

var payload = "";
var data = []; // Array to store payload data from UDP datagram

var http = require('http').Server(app);

var rimraf = require("rimraf");
//rimraf("/Users/johnk/mydb", function () { console.log("done"); });

rimraf.sync("/Users/johnk/mydb");


//Define paths
//const publicDirPath = path.join(__dirname, "../public");
//this is optional (allows you to call views directory by another name) Make sure you use app.set('views',viewsPath)
//const viewsPath = path.join(__dirname, "../public");

//express needs the first key to be view engine and second to be hbs
//this sets up handlebars
//app.set("view engine", "hbs");
//app.set("views", viewsPath);
//app.use(express.static(publicDirPath));

// Global variable to keep track of LED Button status
//let ledButtonPress = 0;

// Global variables for sensor data
// let thermistorData = 0;
// let accelXData = 0;
// let accelYData = 0;
// let accelZData = 0;
// let accelRollData = 0;
// let accelPitchData = 0;

let fob_ID = 0;
let vote = 0;

//flags
var id1 = 0;
var id2 = 0;
var id3 = 0;
var id4 = 0;
var id5 = 0;
var id6 = 0;
var id7 = 0;
var id8 = 0;
var id9 = 0;

// On connection, print out received message


  // Send Ok acknowledgement

  

// Bind server to port and IP


//FOR HTML and web client configuration
app.get("/", function (req, res) {
  res.sendFile(__dirname + '/table.html');
});

app.post("/", function (req, res) {
  res.sendFile(__dirname + '/table.html');
});

var http = require('http').Server(app);
http.listen(9002, function(){
  console.log('graphing at port 9002');
});

var io = require('socket.io')(http);
io.on('connection', function(socket){
  io.emit('fob1',fob1);
  io.emit('fob2',fob2);
  io.emit('fob3',fob3);
  io.emit('fob4',fob4);
  io.emit('fob5',fob5);
  io.emit('fob6',fob6);
  io.emit('fob7',fob7);
  io.emit('fob8',fob8);
  io.emit('fob9',fob9);
});

setInterval(function(){
  io.emit('fob1', fob1);
  io.emit('fob2', fob2);
  io.emit('fob3', fob3);
  io.emit('fob4', fob4);
  io.emit('fob5', fob5);
  io.emit('fob6', fob6);
  io.emit('fob7', fob7);
  io.emit('fob8', fob8);
  io.emit('fob9', fob9);
}, 100);


var db = require('./db');

var information1 = {};
var information2 = {};
var information3 = {};
var information4 = {};
var information5 = {};
var information6 = {};
var information7 = {};
var information8 = {};
var information9 = {};
var information;


var time1 = [];
var time2 = [];
var time3 = [];
var time4 = [];
var time5 = [];
var time6 = [];
var time7 = [];
var time8 = [];
var time9 = [];

var fobID1 = [];
var fobID2 = [];
var fobID3 = [];
var fobID4 = [];
var fobID5 = [];
var fobID6 = [];
var fobID7 = [];
var fobID8 = [];
var fobID9 = [];

var vote1 = [];
var vote2 = [];
var vote3 = [];
var vote4 = [];
var vote5 = [];
var vote6 = [];
var vote7 = [];
var vote8 = [];
var vote9 = [];

var fob1 = {};
var fob2 = {};
var fob3 = {};
var fob4 = {};
var fob5 = {};
var fob6 = {};
var fob7 = {};
var fob8 = {};
var fob9 = {};
// Data Routes

// Port and IP
var PORT = 9002; //port
var HOST = "192.168.1.186"; //need host ip here

// Create socket
var server = dgram.createSocket("udp4");

// Create server
server.on("listening", function () {
  var address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});

server.on("message", function (message, remote) {
  console.log(remote.address + ":" + remote.port + " - " + message);
  var str = message.toString();
  payload = str.split(",");
  readData(payload);
  //console.log("DATA READ___________________________");
  DatabaseSend();
  console.log("Payload: ", payload);

});

server.bind(PORT,HOST);


//read informaiton from esp32 and put it into an array
function readData(payload){
  
    
      information = parseInt(payload[0]);
      var ti = new Date();
      time = ti.toLocaleString();

          switch(information) {
          case 0:

            time1.push(time);
            vote1.push(payload[1]);
            fobID1.push("0");
            

              information1 = {
                  time: time1,
                  vote: vote1,
                  fobID:fobID1
              };
          break;
          case 1:

            time2.push(time);
            vote2.push(payload[1]);
            fobID2.push("1");
            

              information2 = {
                  time: time2,
                  vote: vote2,
                  fobID:fobID2
              };
          break;
          case 2:

            time3.push(time);
            vote3.push(payload[1]);
            fobID3.push("2");
            

              information3 = {
                  time: time3,
                  vote: vote3,
                  fobID:fobID3
              };
          break;
          default:
      
    
  }
}

//Push data to the database
async function DatabaseSend(){
  // store formatted data into the database
      let promise = new Promise((resolve, reject) => {
          setTimeout(() => resolve("done!"), 500)
        });

      let result = await promise; // wait until the promise resolves (*)
     

    if(id1 == 0 && payload[0] == 0){
      db.put(1, information1);
      id1 = 1;
    }
    else if(id2 == 0 && payload[0] == 1){
      db.put(2, information2);
      id2 = 1;
    }
    else if(id3 == 0 && payload[0] == 2){
      db.put(3, information3);
      id3 = 1;
    }
    else if(id4 == 0 && payload[0] == 3){
      db.put(4, information4);
      id4 = 1;
    }
    else if(id5 == 0 && payload[0] == 4){
      db.put(5, information5);
      id5 = 1;
    }
    else if(id6 == 0 && payload[0] == 5){
      db.put(6, information6);
      id6 = 1;
    }
    else if(id7 == 0 && payload[0] == 6){
      db.put(7, information7);
      id7 = 1;
    }
    else if(id8 == 0 && payload[0] == 7){
      db.put(8, information8);
      id8 = 1;
    }
    else if(id9 == 0 && payload[0] == 8){
      db.put(9, information9);
      id9 = 1;
    }
      //db.put(1, information1);
      //db.put(2, information2);
      //db.put(3, information2);

      console.log(information1);
      console.log(information2);
      console.log(information3);
//Conditional. 
      db.get(1, function(err, value) {
              if (err) {
              console.error("null");
              }
              else{
                  console.log("Fob 1:", value);
                  fob1 = value;
              }
        });
      db.get(2, function(err, value) {
              if (err) {
              console.error("null");
              }
              else{
                  console.log("Fob 2:", value);
                  fob2 = value;
              }
          });
      db.get(3, function(err, value) {
              if (err) {
              console.error("null");
              }
              else{
                  console.log("Fob 3:", value);
                  fob3 = value;
              }
          });
      db.get(4, function(err, value) {
            if (err) {
            console.error("null");
            }
            else{
                console.log("Fob 4:", value);
                fob4 = value;
            }
        });
        db.get(5, function(err, value) {
          if (err) {
          console.error("null");
          }
          else{
              console.log("Fob 3:", value);
              fob5 = value;
          }
      });
      db.get(6, function(err, value) {
        if (err) {
        console.error("null");
        }
        else{
            console.log("Fob 6:", value);
            fob6 = value;
        }
    });
    db.get(7, function(err, value) {
      if (err) {
      console.error("null");
      }
      else{
          console.log("Fob 7:", value);
          fob7 = value;
      }
  });
  db.get(8, function(err, value) {
    if (err) {
    console.error("null");
    }
    else{
        console.log("Fob 8:", value);
        fob8 = value;
    }
});
db.get(9, function(err, value) {
  if (err) {
  console.error("null");
  }
  else{
      console.log("Fob 9:", value);
      fob9 = value;
  }
  });
 }

function deleteVotes() {
    rimraf.sync("/Users/johnk/mydb");
}
// const clearVoteButton = document.querySelector('#clearVotes');
// clearVoteButton.addEventListener('submit', (e) => {
//   e.preventDefault()
//   deleteVotes();
// });


//Serve on localhost:9002
//app.listen(9002);
