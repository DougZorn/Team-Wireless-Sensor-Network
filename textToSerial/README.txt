This folder contains the prototype program for the serialToFromText program.

It is the second half, which reads in from a text file for the first line, which contains "-3 roundNumber".  When it detects that roundNumber has increased by 1 from what it saw before, it sends anything in the text file over to serial

Run it in Processing or Processing 2.  Make sure the line in setup that says 

myPort = new Serial(this, Serial.list()[2], 9600); 

has the correct value where the "2" is.  This number depends on how many com ports you are currently using, and which one is the connection you are trying to use.