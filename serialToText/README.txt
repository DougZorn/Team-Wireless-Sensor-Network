This folder contains the prototype program for the serialToFromText program.

It is the first half, which reads in from the serial line and once it sees "-3 roundNumber" it sends anything from the serial line to Text until it sees "-4 roundNumber" which is the conclusion of the message from the command node.

Run it in Processing or Processing 2.  Make sure the line in setup that says 

myPort = new Serial(this, Serial.list()[2], 9600); 

has the correct value where the "2" is.  This number depends on how many com ports you are currently using, and which one is the connection you are trying to use.