# Code Readme

### Fob Code
The fobs are all running the same code with the exception of their respective device IDs, which are unique and can be found on lines 121 and 122. Each fob also makes use of a global ipTable variable, found on line 130, that maps device IDs to their respective IP addresses (The indices correspond with the device ID). We also define a NUM_FOBS constant on line 135 for looping through the device IPs. Due to limited hardware, we used 3 fobs for this demo. 

###  Leader Election Section -------------------------------------------------------------------------------------------

This program was created by combining the udp_client and udp_server code examples and our skill 25 code. 

### State Diagram
The program revolves around three states: ELECTION_STATE, LEADER_STATE, FOLLOWER_STATE (lines 113 - 118). All FOBs enter the election in the ELECTION_STATE and it represents FOBs that are still in contension for becoming the Leader. A FOB is moved to the FOLLOWER_STATE if it realizes that it's ID is higher than the FOBs that is is communicating to which means that it is now out of the race. Lastly, once the election times out, a FOB moves from election to the LEADER_STATE which signifies that it is the declared leader.

### Timers and Timeouts
To keep all the FOBs synced up, we implemented a timer task that decrements timeout variables (lines 515 - 555). We have two timeout variables and 4 timeout constants. The two timeout variables are: a general timeout variable and a udp timeout. The general timeout varibale is dependent on each state and will send a FOB back to election state when it hits zero. The udp timeout is also dependent on the state and tells the FOB to send a UDP message to all the other FOBs when it hits zero. The 4 timeout constants are used to reset the timeout to a specific value and they are the ELECTION_TIMEOUT (15 sec), LEADER_TIMEOUT (30 sec) HEARTBEAT (1 sec) UDP_TIMER (3 sec) (lines 90 - 93). 

### Election State
In the ELECTION_STATE, the timeout is initally set to ELECTION_TIMEOUT and the udp timeout is initally set to UDP_TIMER. This allows the FOB to sends UDP messages to all other FOBs every 3 seconds. Once a FOB receives a message from another FOB, it compares the received ID with the FOB's personal ID. If the personal ID is smaller than the received ID, it resets the timeout back to ELECTION_TIMEOUT and continues to wait for messages. If the received ID is larger, the FOB will move states to the FOLLOWER_STATE and stop transmitting messages (lines 799 - 803). Once there is only one FOB left, the timeout will go to zero which sends the FOB to the LEADER_STATE (lines 531 - 535).

### Follower State
In the FOLLOWER_STATE, the FOB knows that it is out of the election race. It stops sending udp client messages and just listens for heartbeats from a leader. The timeout is initally set to LEADER_TIMEOUT which is 30 seconds and if it receives a udp message that has the status leaderHeartbeat with the content of "Alive", it will reset the timeout to 30 seconds and continue listening (lines 543 - 552). If a leader is broken / removed and stops sending udp heartbeat messages, the timeout varibale will reach zero and return to the ELECTION_STATE. Another scenario where the FOB can go back to ELECTION_STATE is if a new device is connected since it will send a UDP message with the status of deviceAge of "New". The FOB will recognize the new device and restart the election (lines 810 - 813).

### Leader State
In the LEADER_STATE, the FOB disregards the general timeout varibale and only uses the udp timeout. It is set to HEARTBEAT and sends a udp message every 1 second that states the leaderHeartbeat status is "Alive" (lines 543 - 552). It will only change states if a new device is connected since it will send a UDP message with the status of deviceAge of "New". The FOB will recognize the new device and restart the election (lines 828 - 835).

### UDP Payload
Each FOB sends a UDP message when the UDP timer arrives at zero, and it contains information about the status, myID, deviceAge, and leaderHeartbeat (lines 912 - 960). It is then parsed at the receiving server using strtok and delimiting using "," (lines 651 - 689). 

###  Leader Election Section End -------------------------------------------------------------------------------------------

The majority of the election handling is done inside the UDP client and server tasks. The "udp_server_task" on line 560 uses a series of conditionals to handle state changing of the fob after receiving leader election data over UDP. On line 655, a for loop deconstructs the received payload, determines if it contains leader election data or poll data, and stores its contents in related variables. 

Once a poll leader has been elected, the actual voting process can begin. The fobs toggle between voting red or blue inside button_task() on line 346. Pressing the button changes the colorID variable, which is then checked with a switch case inside led_task() on line 464. This LED task checks the state of the colorID variable, and will turn on the appropriate red or blue LED on the fob.

After choosing to vote red or blue, a second button is pressed to transmit the fob ID and it's vote ('R' or 'B') over NFC/IR to an adjacent fob. The second button trips a flag (sendFlag) inside of button_2_task() on line 373. Conceptually, we chose to only transmit voter data over NFC/IR if the fob is not in a leader state. We chose to do this because we felt that the poll leader should have the ability to transmit its vote directly to the node.js server over UDP, rather than having to transmit to an adjacent fob, receive its own vote back, and only then being able to transmit to the node.js server.

Once the vote has been sent over IR, the receiving fob will "blink" the registered vote to show that it has been received. This happens on lines 432 - 441. The correct led pins that correspond to the received vote are set to a value of 1 and then turns off on the next cycle of led_task.

So, if the fob is not in a leader state (Is not the poll leader), and the sendFlag is tripped, it will enter a conditional inside the recv_task() on line 397. The code inside this conditional will then transmit the fob ID and vote to an adjacent fob over NFC/IR. The adjacent fob will then have the RecvFlag tripped on line 442 upon receiving the IR data. If the adjacent fob is not the poll leader, a conditional will then execute inside the udp_client_task() on line 1045, which handles sending the original fob's ID and vote to the poll leader over UDP. If the adjacent fob happens to be the poll leader already, a conditional inside the udp_client_task() on line 961 will instead execute, sending the original fob's ID and vote directly to the node.js server over UDP.

If the poll leader is sending its own vote, the conditional on line 1120 will execute, which sends its own fob ID and vote directly to the node.js server over UDP.


### Votes.js Code
This is our nodejs program that handles receiving udp packets from the c program, parsing the information, and storing it to the database and sending it to the front end. Lines 76-82 handle our html and web client configuration with app.get and app.post /table.html. This program uses sockets to emit information on the front end. Once we establish a connection we created a 9 fob functions that will include time, FobID, and vote objects. Additionally, every fob will have it's own time, ID, and will be able to cast their own vote. 

Lines 185-192 receive the message from the c program and parse the data into payload which will have the fobID and the fob's respective vote. 

Lines 200 - 251 is our readData function. This takes in argument payload and depending on which fob we are looking at we push the time, ID, and vote into information{}. 

Lines 254-388 is our Database send function. Lines 254 - 305 are going to take the information we just created for each fob and store it in the created database using a db.put('fobID', information#). Whereas lines 306 - 388 use db.get to but all the information into each respective fob to be used in the front end. 

Finally, our last function is deleteVote() on line 390 which uses the rimraf module to synchronously delete our database directoy so we can store new user votes simulating a new election. This function is called in the front end on a button click in table.html.

### Front-End table.html
This is our front end program for the webpage "Voting Hype!" This program uses sockets and jquery to access information from node server. Lines 23 to 241 include the fob queries for up to 9 fobs from the node program to access time, fobID and vote. Once each socket is created we creat a table with the Date and Time, FobID, and Vote. This is the same for all 9 fobs and can first be seen on lines 27-30. From that we populate each table column with each fob object (fob.time, fob.FobID, fob.vote). First iteration of this can be seen on lines 35-37. 

Additionally, after each fob's objects are created and populated into the table, we check what the fob.vote is and increment either blue votes or red votes. This happens after each fob table is setup. The first iteration is on lines 43- 50.

Finally, towards the end of the program is where we have our buttons setup to count the votes and delete the ballots. These use the onclick button feature and are called on lines 248 and 249 respectively. 

### Database db.js code
This file sets up our leveldb database to store all of our voter data. This file is imported into votes.js so we can put information into the database. The directory name to be created in the pi is on line 10 and the value encoding is 'json' on line 13.
