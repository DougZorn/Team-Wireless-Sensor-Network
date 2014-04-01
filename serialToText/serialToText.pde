import processing.serial.*;
Serial myPort;  
File ff = new File("input.txt");
PrintWriter outputFromA;
boolean endReached = false;
int rounds = 1;
String input = null;
String[] data;

void setup()
{
  // List all the available serial ports:
  println(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[2], 9600);                          //Pick Arduino serial port
  myPort.clear();
  outputFromA = createWriter("input.txt");
}

void draw() {
  readFromSerial();                                                           //Create new text file for R input 
}
void keyPressed() {
  if (key == 'd')                                                             //Exit program is 'd' ket is pressed
    outputFromA.close();
  exit();
}

void readFromSerial() {
  if (myPort.available() > 0) {                                               //Check for new data in serial port
    input = myPort.readStringUntil('\n');                                     //Store first line of serial data
    if (input != null)
      data = split(input, ' ');                                               //Parse string into parts seperated by spaces
    if (data != null) {                                                       //to prevent NullPointerException errors
      if(int(data[0]) == -3 && int(trim(data[1])) == 1) rounds = 1;      //If node is reset, reset rounds to 1
      if (int(data[0]) == -3 && int(trim(data[1])) == rounds) {               //check if first line is -3 followed by rounds passed '
        if(rounds > 1){                                                           
          ff.delete();                                                        //If new round, delete file to erase last round and recreate file   
          File ff = new File("input.txt");
          outputFromA = createWriter("input.txt");
        }
        println("New Round");
        endReached = false;                                                   //Reset round
        printToFile(data);
        while (endReached == false) {                                         //While there is new data, store it in input array
          if (myPort.available () > 0) {
            input = myPort.readStringUntil('\n');                             //Read data from serial line by line
            data = split(input, ' ');
            if (data != null) {
              println("Print to file");  
              printToFile(data);                                              //Print received data to file
              if (int(data[0]) == -4) {                                       //Combination of -4 followed by number of rounds passed signifies end 
                if (int(trim(data[1])) == rounds) {
                  endReached = true;                                          //End of round, increment rounds and  exit loop
                  println("end of round");
                  rounds++;
                }
              }
            }
          }
          println("done with if (myPort.available () > 0) "); //leave these for now, freezes without these statments, it may need this delay
        }  
        println("done with while (endReached == false)");     //leave these for now, freezes without these statments, it may need this delay
      }
    }
  }
}

void printToFile(String[] data) {
  int h = 0;
  outputFromA.print(data[h]);  
  println(data[h]); 
  for (h = 1; h < data.length; h++) {                             //Enter loop to print data array to text file 
    outputFromA.print(" "+ data[h]);                              //Print value to file
    println(data[h]);
  }
  outputFromA.flush();                                            //Flush data to print to text file
}
