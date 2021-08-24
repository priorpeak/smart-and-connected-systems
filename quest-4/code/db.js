//John Kircher, Alex Prior, Allen Zou
//This program sets up the database to store our time, fob_ID, and vote
//When we import thise to votes.js it will put items into the database
//using tingoDB./

// import libraries
var level = require('level');
var path = require('path');
// create a database folder to store data
var dbPath = path.join(__dirname, 'mydb');
// store values that are jsons
var db = level(dbPath, {
    valueEncoding: 'json'
});
// export database
module.exports = db;
