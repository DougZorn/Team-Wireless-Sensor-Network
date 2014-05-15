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
  myPort = new Serial(this, Serial.list()[1], 9600);                          //Pick Arduino serial port
  myPort.clear();
}


void draw() {
  switch(state) {
  case INSPECTSERIAL:
   // println("inspectSerial state");

    if (myPort.available() > 0) {
      serialInput = myPort.readStringUntil('\n');
      if (serialInput != null) {
        serialPieces = split(serialInput, ' ');
        if (int(trim(serialPieces[0])) == -3) {
          //if rounds haven't started yet, start them by saving current round number
          if (rounds <= 0) {
            state = READSERIAL;
            //mess with File file and file.delete()?
            rounds = int(serialPieces[0]);
            println("Round Number from Serial: " + rounds);
          }
          //otherwise only valid when what you receive is 1 more than the previous round
          else if (int(trim(serialPieces[1])) == rounds+1) {
            state = READSERIAL;
            //mess with File file and file.delete()? stub
            rounds = int(serialPieces[0]);
            println("Round Number from Serial: " + rounds);
          }
          //otherwise something went wrong with the round numbers
          else {
            println("Error: Expected round + 1 from Serial, found " + int(pieces[1]));
          }
          //Starting or continuing, you always need to start a new file before printing to a text file
        }
      }
    }
    break;


  case READSERIAL:
    //println("readSerial state");
    writer = createWriter("Serialoutput.txt");

    print(int(serialPieces[0]));
    print(" ");
    println(int(serialPieces[1]));

    writer.print(int(serialPieces[0]));
    writer.print(" ");
    writer.print(int(serialPieces[1]));

    while (myPort.available () > 0) {
      serialInput = myPort.readStringUntil('\n');

      if (serialInput != null) break;
      serialPieces = split(serialInput, ' ');

      printToText(serialPieces);

      if (int(trim(serialPieces[0])) == -4) break;
    }

    writer.flush();
    writer.close();
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
        if (rounds <= 0) {
          state = READTEXT;
          rounds = int(pieces[1]);
          println("Round Number from text: " + rounds);
        }
        //else check for round being one more than the previous round
        else if (int(trim(pieces[1])) == rounds+1) {
          state = READTEXT;
          rounds = int(pieces[1]);
          println("Round Number from text: " + rounds);
        }
        //otherwise something went wrong with the Round numbers
        else {
          println("Error: Expected round + 1 from Text, found " + int(pieces[1]));
        }
      }
    }
    delay(100);
    break;


  case READTEXT:
    //println("readText state");
    //Already have the first line, print it
    print(int(pieces[0]));
    print(" ");
    println(int(pieces[1]));

    myPort.write(int(pieces[0]));
    myPort.write(" ");
    myPort.write(int(pieces[1]));

    //Loop through the rest of text, print and print to serial
    while (line!= null) {
      line = getNextLine(reader);
      if (line == null) break;
      pieces = split(line, " ");

      if (haveNewOrders && int(trim(pieces[0])) == -4) {
        for(int i = 0; i < ordersArray.length; i++){
          myPort.write(trim(ordersArray[i]));
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
    println("default state");
    break;
  }
}

void printToText(String[] in) {
  for (int i = 0; i < in.length; i++) {
    writer.print(int(in[i]));
    print(int(in[i]));
    if (i  != in.length-1){
      writer.print(" ");
      print(" "); 
      }
  }
  println("");
  //possibly flush here? stub
}

void printToSerial(String[] in) {
  for (int i = 0; i < in.length; i++) {
    myPort.write(int(in[i]));
    print(int(in[i]));
    if (i  != in.length-1){
      myPort.write(" ");
      print(" ");
    }
  }
  println("");
}


void delay(int amount) {
  int startTime = millis();
  while (startTime + amount > millis ()) {
    //Do Nothing.
  }
  return;
}


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


void keyPressed() {
  keyInput = append(keyInput, key); 
  if (key == ENTER) {
    haveNewOrders = true;

    //compile entered keys into a string called "completeCommand"
    completeInput = emptyStringArray;
    for (int i = 0; i < keyInput.length; i++) {
      completeInput = append(completeInput, str(keyInput[i]));
    }
    //STUB can strings be interpretted by the command node, or should I convert to a number, then convert back in the command node?
    String completeCommand = "-5 " + join(completeInput, "");
    ordersArray = append(ordersArray, completeCommand);
    println(completeCommand);

    for(int i = 0; i < ordersArray.length; i++){
      print("{");
      print(trim(ordersArray[i]));
      println("}");
    }

    //..and empty array of entered input
    keyInput = emptyCharArray;
  }
  else {
    //Print entire set of entered keys since last ENTER key hit
    println(keyInput);
    println("");
  }
}

