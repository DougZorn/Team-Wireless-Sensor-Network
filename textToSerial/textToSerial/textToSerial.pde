import processing.serial.*;
Serial myPort;

BufferedReader reader;
String line;
int rounds = -1;

void setup() {
    println(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[2], 9600);                          //Pick Arduino serial port
  myPort.clear();
}

boolean flag = false;
String[] pieces;

void draw() {

  //Look for new round, or start on first round
  while (!flag) {
    reader = createReader("input.txt");
    line = getNextLine(reader);

    myPort.write(65);

    if (line != null) {
      pieces = split(line, ' ');
      if (int(trim(pieces[0])) == -3) {
        //if you haven't started checking rounds yet, just accept and go
        //else check to see if it's the next round from the last loop
        if (rounds == -1) {
          flag = true;
          rounds = int(pieces[1]);
          println("Round Number: " + rounds); 
        } else {
          if (int(trim(pieces[1])) == rounds+1) {
            flag = true;
            rounds = int(pieces[1]);
            println("Round Number: " + rounds); 
          }
        }
      }
    }
    //If you aren't starting or you don't detect the next round, then wait a bit and check next time
    delay(1000);
  }

  while (line!= null) {
    //first line, no splitting necessary
    if (flag) {
      print(int(pieces[0]));
      print(" ");
      println(int(pieces[1]));
      
      //Resetting flag
      flag = false;
    }else{
       line = getNextLine(reader);
      if(line == null) break;
       pieces = split(line, " ");
      
      if(int(trim(pieces[0])) == -2){
        print(int(pieces[0]));
        print(" ");
        print(int(pieces[1]));
        print(" ");
        print(int(pieces[2]));
        print(" ");
        println(int(pieces[3]));
      }else if(int(trim(pieces[0])) == -4){
        print(int(pieces[0]));
        print(" ");
        println(int(pieces[1]));
      }else{
        print(int(pieces[0]));
        print(" ");
        print(int(pieces[1]));
        print(" ");
        println(int(pieces[2]));
      }
    }
  }
}

void delay(int amount) {
  int startTime = millis();
  while (startTime + amount > millis ()) {
    //Do Nothing
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

