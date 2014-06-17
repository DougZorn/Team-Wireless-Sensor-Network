This is the program which connects R to the command node through the serial line and a few text files.

The command node starts a round in the network, then when all slave nodes have gone, the command node collects the data.  This data is sent through the Serial line with the first line always as "-3 roundNumber" and the last line always as "-4 roundNumber".

This program periodically reads the serial line, and when it sees a line that first "-3 roundNumber" line, it picks up everything from the serial line and puts it into a text file, until it reads a "-4 roundNumber".

At this point, R picks up the file as input, processes it, and creates another file as output. R increments the roundNumber in the first line.

While R is working, this processing program periodically looks at R's output file, waiting for the roundNumber to change in the first round.  When this value becomes 1 higher than it was previously, the program reads in the entire text file into the serial line.

This program repeats this cycle: Waiting and reading from serial until triggered, sending from serial to text, waiting and reading from text until triggered, then sending from text to serial.

Additionally, Processing allows for text input.  An interrupt based keypress detector picks up any key that is pressed, logs it, and when you hit <enter>, it checks to see if it's a valid command, and if so, splices that command into the next outgoing message from text to serial.  This command is formatted as: "-5 command" and this format is what is parsed in by the command node.

The other two Processing programs in this repository (serialToFromTextArduinoTester and serialToFromTextProcessingTester) are for debugging and learning

