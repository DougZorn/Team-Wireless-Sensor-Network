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
  myPort = new Serial(this, Serial.list()[2], 9600);                          //Pick Arduino serial port
  myPort.clear();
}


void draw() {
  switch(state) {
  case INSPECTSERIAL:
    println("inspectSerial state");

    if (myPort.available() > 0) {
      serialInput = myPort.readStringUntil('\n');
      println("received: " + serialInput);
      
      if (serialInput != null) {
        serialPieces = split(serialInput, ' ');
        
        if (int(trim(serialPieces[0])) == -3) {
          
            state = READSERIAL;
            //mess with File file and file.delete()? stub
            rounds = int(trim(serialPieces[1]));
            println("Round Number from Serial: " + rounds);
        }
      }
    }
    break;


  case READSERIAL:
    println("readSerial state");
    writer = createWriter("Serialoutput.txt");

    print(int(serialPieces[0]));
    print(" ");
    println(int(trim(serialPieces[1])));

    writer.print(int(serialPieces[0]));
    writer.print(" ");
    writer.println(int(trim(serialPieces[1])));

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
      
      printToText(serialPieces);
      
       //-4 check is necessary to ensure we don't pick up other junk in Serial. We only want what's from -3 to -4
      if (int(trim(serialPieces[0])) == -4) break;
    }
    
    println("writing to file");

    writer.flush();
    writer.close();
    
    
    //state = INSPECTSERIAL;
    state = INSPECTTEXT;
    break;


  case INSPECTTEXT:
    println("inspectText state");
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
    println("readText state");
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
    writer.print(trim(in[i]));
    print("printing: ");
    println(trim(in[i]));
    if (i  != in.length-1){
      writer.print(" ");
      print(" "); 
      }
  }
  writer.println("");
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
  if (key == ENTER) {
    haveNewOrders = true;

    //compile entered keys into a string called "completeCommand"
    completeInput = emptyStringArray;
    for (int i = 0; i < keyInput.length; i++) {
      completeInput = append(completeInput, str(keyInput[i]));
    }

    //test for valid command
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

