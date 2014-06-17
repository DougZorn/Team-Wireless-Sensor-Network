import processing.serial.*;

final int INSPECTSERIAL = 1;
final int READSERIAL = 2;
final int INSPECTTEXT = 3;
final int READTEXT = 4;

//State Machine. 
//->First by looking for change in serial
int state = INSPECTSERIAL;

//Reading from Text variables
BufferedReader reader;
String line;

//Reading from Serial variables
PrintWriter writer;
File file = new File("testingFileFunction.txt");
boolean endReached = false;
String input = null;
String serialInput;
String[] serialPieces;

//Keyboard input variables
String[] pieces;
char[] keyInput = {
};

//General variables
int rounds = -1;
Serial myPort;

//Transmitting Typed Commands
boolean haveNewOrders = false;
String[] ordersArray = {};
String[] emptyStringArray = {};
String[] completeInput = {};
char[] emptyCharArray = {};

void setup() {
  println(Serial.list());
  // Open the port you are using at the rate you want: 
  //put Serial.list()[here] to 1 when no node is plugged in, 2 when there is a node
  myPort = new Serial(this, Serial.list()[3], 9600);                          //Pick Arduino serial port
  myPort.clear();
}


void draw() {
  switch(state) {
  case INSPECTSERIAL:
    //println("inspectSerial state");

	//Anytime there's anything in the serial channel, read it
    if (myPort.available() > 0) {
      serialInput = myPort.readStringUntil('\n');
      println("Received: " + serialInput);
      
      if (serialInput != null) {
        serialPieces = split(serialInput, ' ');
        
        if (int(trim(serialPieces[0])) == -3) {
			//Only when you read in a line that starts with "-3" do you move on
            state = READSERIAL;
            rounds = int(trim(serialPieces[1]));
            println("Round Number from Serial: " + rounds);
        }
      }
    }
    break;


  case READSERIAL:
    //println("readSerial state");
	//Create a text file
    writer = createWriter("Serialoutput.txt");
	
	//Manually write the first line, since you already have it
    print(int(serialPieces[0]));
    print(" ");
    println(int(trim(serialPieces[1])));

    writer.print(int(serialPieces[0]));
    writer.print(" ");
    writer.println(int(trim(serialPieces[1])));

	//Then read through all that is in the serial channel and put it into the text file
    while (myPort.available () > 0) {
      delay(10);
      serialInput = myPort.readStringUntil('\n');
      println("received: " + serialInput);

      if (serialInput == null) {
        print("break from serialInput null");
        break;
      }
        
      serialPieces = split(serialInput, ' ');
      println("pieces: " + serialPieces[0] + " " + serialPieces[1]);
      
	  //Helper program prints to text in proper format
      printToText(serialPieces);
      
       //-4 check is necessary to ensure we don't pick up other junk in Serial. We only want what's between -3 and -4
      if (int(trim(serialPieces[0])) == -4) break;
    }
    
    println("writing to file");

	//These two lines are required to finish text file
    writer.flush();
    writer.close();
    
    
    //state = INSPECTSERIAL;  //uncomment this line to only read from serial to text
    state = INSPECTTEXT;  //For normal use, use this line to start waiting on the text file
    break;


  case INSPECTTEXT:
    //println("inspectText state");
    //Look for new round in first line of text
    reader = createReader("Routput.txt");
    line = getNextLine(reader);

    if (line != null) {
      //split first line of text into [-3][round number]
      pieces = split(line, ' ');
      if (int(trim(pieces[0])) == -3) {
        //if you haven't started checking rounds yet, grab current round number
        if (int(trim(pieces[1])) == rounds) {
          state = READTEXT;
          rounds = int(pieces[1]);
          println("Round Number from text: " + rounds);
        }
        //otherwise something went wrong with the Round numbers
        else {
          println("Waiting for round " + rounds + ", text is on round " + int(pieces[1]));
        }
      }
    }
    break;


  case READTEXT:
    //println("readText state");
    //Already have the first line, so print it to serial manually
    print(int(pieces[0]));
    print(" ");
    println(int(pieces[1]));

    myPort.write(trim(pieces[0]));
    myPort.write(" ");
    myPort.write(trim(pieces[1]));
    myPort.write("\n");

    //Loop through the rest of text, print to serial
    while (line!= null) {
      line = getNextLine(reader);
      if (line == null) break;
      pieces = split(line, " ");

	  //This either prints to serial normally, or if there is are additional commands that have been typed in
	  //those commands get written right before the last, "-4", line gets printed
      if (haveNewOrders && int(trim(pieces[0])) == -4) {
        for(int i = 0; i < ordersArray.length; i++){
          myPort.write(trim(ordersArray[i]));
          print("Sending Order: ");
          println(trim(ordersArray[i]));
        }
        
        printToSerial(pieces);
        ordersArray = emptyStringArray;
      }else{
        printToSerial(pieces);
      }
    }
	
    state = INSPECTSERIAL;
    break;

  default:
	//Something broke if you're here.
    println("default state");
    break;
  }
}

//Helper program to print to text in proper format.  Also prints to Processing terminal
void printToText(String[] in) {
  println("Printing line to text: ");
  for (int i = 0; i < in.length; i++) {
    writer.print(trim(in[i]));
    print(trim(in[i]));
    if (i  != in.length-1){
      writer.print(" ");
      print(" "); 
      }
  }
  writer.println("");
  println("");
}

//Helper program to print to Serial in proper format.  Also prints to Processing terminal
void printToSerial(String[] in) {
  println("printing line to Serial: ");
  for (int i = 0; i < in.length; i++) {
    myPort.write(trim(in[i]));
    print(trim(in[i]));
    if (i  != in.length-1){
      myPort.write(" ");
      print(" ");
    }
  }
  myPort.write("\n");
  println("");
}

//Blocking delay function, which helps at times
void delay(int amount) {
  int startTime = millis();
  while (startTime + amount > millis ()) {
    //Do Nothing.
  }
  return;
}

//Convenience helper program
String getNextLine(BufferedReader br) {
  String line;
  try {
    line = reader.readLine();
  } 
  catch (IOException e) {
    e.printStackTrace();
    line = null;
  }
  return line;
}

//Interrupt based keypress checker, which also processes and parses in the char arrays when you hit <enter>
void keyPressed() { 
  if (key == ENTER) {
    haveNewOrders = true;

    //Compile entered keys into a string called "completeCommand"
    completeInput = emptyStringArray;
    for (int i = 0; i < keyInput.length; i++) {
      completeInput = append(completeInput, str(keyInput[i]));
    }

    //Test for valid command
    String[] test = split(join(completeInput,""), ' ');
    println("test[0] is: {" + test[0] + "}");
    if(trim(test[0]).equals("shutdown") || test[0].equals("node") || test[0].equals("flight") || test[0].equals("land")){
      String completeCommand = "-5 " + join(completeInput, "");
      ordersArray = append(ordersArray, completeCommand);
      println("List of orders:");
      println(completeCommand);
    }else{
      println("Typo: you typed invalid command {" + join(completeInput,"") + "}");
    }
    
    for(int i = 0; i < ordersArray.length; i++){
      print("{");
      print(trim(ordersArray[i]));
      println("}");
    }

    //..and empty array of entered input
    keyInput = emptyCharArray;
  }
  else {
    keyInput = append(keyInput, key);
    //Print entire set of entered keys since last ENTER key hit
    for(int i = 0; i < keyInput.length; i++){
      print(keyInput[i]);
    }
    println("");
  }
}

