/*  
 *  Command Node Code
 *  
 *  Before using, check:
 *  NUM_NODES, MY_NAME (should always be 0)
 *  Anchor node locations, x1, y1, x2, y2, x3, y3
 *
 */

#include <SPI.h>
#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

//Number of nodes, including Command Node
const byte NUM_NODES = 4;
byte RST[PACKET_LENGTH];

//The LED PIN
int ledPin = 9;

void setup(){
  Serial.begin(9600);
  init_CC2500_V2();
  pinMode(ledPin,OUTPUT);
  while(listenForPacket(RST)){
  }

  digitalWrite(ledPin, HIGH);
  for(int i = 0; i < 5; i++){
    sendPacket(0, 255, 0, 0, 200, 0);
  }
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);

  //This just hardcodes some values into the Desired table, as bytes
  randomSeed(analogRead(3));
  int desired[NUM_NODES][2];
  for(int i = 0; i < NUM_NODES; i++){
    desired[i][0] = roundUp(random(30,230));  //roundUp((rand() % 200) + 30);  //gets a range of randoms from -100ish to 100ish
    desired[i][1] = roundUp(random(30,230)); //roundUp((rand() % 200) + 30);
  }
  desired[0][0] = byte(128);
  desired[0][0] = byte(128);

}


//State names
const int INIT = 0;
const int RECEIVE = 1;
const int CALCULATE = 2;
int state = INIT;

//used for user Arguments, it will make command node stuck at userCom until further commands
int stuckUser;

//Names of Node and nodes before it
//Determines when this node's turn is
const byte MY_NAME = 0; //Command node is always 0
const byte PREV_NODE = NUM_NODES - 1;
const byte PREV_PREV_NODE = NUM_NODES - 2;

//Initial conditions of three points, first is command node at origin
//stub convert to (-127)-128 before sending to serial
const int x1 = 0;  //128;  //0
const int y1 = 0;  //128;  //0
const int x2 = 12;  //129;  //1
const int y2 = 36;  //131;  //3
const int x3 = 36;  //127;  //turns into -1
const int y3 = -12;  //124;  //turns into -4

//How many data entries to take an average from, arbitrarily set to 15 stub
const int STRUCT_LENGTH = 15;

//The indexes of where each piece of data is (for readability of code)
const int SENDER = 0;
const int TARGET = 1;
const int DISTANCE = 2;
const int SENSOR_DATA = 3;
const int HOP = 4;
const int END_BYTE = 5;
const int RSSI_INDEX = 6;

//This is how many times to resend all data, for redundancy.  Arbitrarily set to 4
const int REDUNDANCY = 4; 

//Timer info
const unsigned long TIMEOUT_PP = 2000; //??? check this timeout number stub
const unsigned long TIMEOUT_P = 1000;

//Global variables
//These control timeouts
unsigned long currTime;
unsigned long lastTime;

//These are the structures that contain the data to be averaged
byte rssiData[NUM_NODES][STRUCT_LENGTH] = {
  0};

//Each contains a pointer for each of the nodes, indicating where to write in the above tables
int rssiPtr[NUM_NODES] = {
  0};

//Arrays of averages
byte rssiAvg[NUM_NODES] = {
  0};  

//The main matrices
byte distances[NUM_NODES][NUM_NODES] = {
  0};
byte allSensorData[NUM_NODES];    //Collected sensor data from all nodes
byte currLoc[NUM_NODES][2] = {
  0}; //The contents of the R calculations file will go here
byte desired[NUM_NODES][2] = {
  0};

//Flags for controlling getting new data every cycle or not
boolean wantNewMsg = true;
boolean gotNewMsg = false; 

//The current message, and storage for data restoration in case of bad packet
byte currMsg[PACKET_LENGTH] = {
  0};
byte oldMsg[PACKET_LENGTH] = {
  0};

//Current Round number
int roundNumber = 0;

//Temporary variable used for averaging
unsigned int temp = 0;
int goodMsg = 0;

//Helps coordinate timeout
byte lastHeardFrom;

typedef struct {                    //array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            //smoother version of accADC
  int16_t  gyroData[3];             //Not sure
  int16_t  magADC[3];              //180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             //raw gyro data
  int16_t  accADC[3];              //raw accelerometer data
} 
imu_t;

typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;
} 
analog_t;

typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} 
alt_t;

typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} 
att_t;

/*
//This function converts Serial values which can be negative into positive bytes
 int byteToInt(byte input){
 if(input > 128){
 input = int(input - byte(128));
 return input;
 }else if(input < 128){
 input = int((byte(128) - input)) * -1;
 return input;
 }else return 0;
 }
 */

/*
//This function converts 
 int byteToInt(byte input){
 if(input > 127){
 return int(255 - input);
 }else return int(input);
 }
 */

//Round values up to nearest whole number, cast to byte
byte roundUp(float input){
  float output = input + 0.9999;
  return byte(output);
}


void userCom(){
  char userCommand[500];
  char snippedCommand[500];
  char *tempCommand1, *tempCommand2;
  char *arg1, *arg2, *arg3, *arg4;
  boolean data_command_flag = true;
  int sequenceNumber;
  int endSequenceNumber;
  int nodeNum;

  digitalWrite(ledPin, LOW);
  if(Serial.available()){
    arg1 = NULL;
    arg2 = NULL;  //use for later improve arguments
    for(int x =0; x<500;x++){
      userCommand[x] = NULL;
      snippedCommand[x] = NULL;
    }
    int x =0;
    delay(200);
    do{
      userCommand[x++] = Serial.read();
    }
    while(Serial.available());

    // delay(200);
     Serial.print("I got ");
     Serial.print(userCommand);
     Serial.print("\n");

    Serial.print("State 1\n");

    
     int mostRecent3 = 0;
     int secondMostRecent3 = 0;
     int mostRecent4 = 0;
     
     for(int i = 0; userCommand[i+1] != NULL; i++){
     if(userCommand[i] == '-' && userCommand[i+1] == '3') {
     secondMostRecent3 = mostRecent3;
     mostRecent3 = i;
     }
     if(userCommand[i] == '-' && userCommand[i+1] == '4') mostRecent4 = i;
     }
     
     //you have a partial message, so you need the previous full message
     if(mostRecent3 > mostRecent4) {
     mostRecent3 = secondMostRecent3;
     Serial.print("Partial message error");
     }
     
     int countX = 0;
     for(int i = mostRecent3; i < mostRecent4; i++){
     snippedCommand[countX++] = userCommand[i];
     }
     
    Serial.print("snippedCommand = ");
    Serial.print(snippedCommand);
    Serial.print("\n");
     
    Serial.print("mostRecent3 = ");
    Serial.print(mostRecent3);
    Serial.print("\n");
    
    Serial.print("mostRecent4 = ");
    Serial.print(mostRecent4);
    Serial.print("\n");
    
    Serial.print("secondMostRecent3 = ");
    Serial.print(secondMostRecent3);
    Serial.print("\n");


     
     Serial.print("State 2\n");
     //arg1 is line
     //arg2 is first entry in line
     

  
  
  

    Serial.print("userCommand = ");
    Serial.print(userCommand);
    Serial.print("\n");


    arg1 = strtok_r(snippedCommand, "\n", &tempCommand1);

    int lineCounter = 0;
    while(arg1 != NULL){
      
      Serial.print("in while loop #");
      Serial.print(lineCounter);
      Serial.print("\n");
      
      Serial.print("I got arg1 = ");
      Serial.print(arg1);
      Serial.print("\n");

      
      
      
      // strtok again is the problem of this
      arg2 = strtok_r(arg1, " ", &tempCommand2);

      Serial.print("I got arg2 = ");
      Serial.print(arg2);
      Serial.print("\n");

      if(!strcmp(arg2, "-1")){
        //handle next/all
        //This is where you'd parse in different values for new nodes
        //or for new anchor node locations, but we aren't having mobile anchor nodes
        //nor live additions of nodes in the network at this point

      }
      else if(!strcmp(arg2, "-3")){

        Serial.print("State Got -3\n");

        //handle next/all
        sequenceNumber = atoi(strtok_r(NULL, " ", &tempCommand2));
        
        Serial.print("sequenceNumber = ");
        Serial.print(sequenceNumber);
        Serial.print("\n");
      }
      else if(!strcmp(arg2, "-4")){
        //handle next/all
        endSequenceNumber = atoi(strtok_r(NULL, " ", &tempCommand2));
        
        Serial.print("State Got -4\n");
        
        Serial.print("endSequenceNumber = ");
        Serial.print(endSequenceNumber);
        Serial.print("\n");

      }
      else if((nodeNum = atoi(arg2)) >= 0){

        //Serial.print("State Got arg>0\n");

        arg3 = strtok_r(NULL, " ", &tempCommand2);
        arg4 = strtok_r(NULL, " ", &tempCommand2);
        
        currLoc[nodeNum][0] = atoi(arg3);
        currLoc[nodeNum][1] = atoi(arg4);
        
        Serial.print("currLoc[nodeNum][0] = ");
        Serial.print( currLoc[nodeNum][0]);
        Serial.print("\n");
        Serial.print("currLoc[nodeNum][1] = ");
        Serial.print(currLoc[nodeNum][1]);
        Serial.print("\n");
        
      }
      else if(!strcmp(arg2, "-5")){
        //handle next/all

        Serial.print("State Got -5\n");

        arg3 = strtok_r(NULL, " ", &tempCommand2);
        arg4 = strtok_r(NULL, " ", &tempCommand2);
        
        String argString = String(arg4);
        int argName = argString.toInt();

        if(!strcmp(arg3, "shutdown") && ((argName<NUM_NODES)||(argName==255))){
          digitalWrite(ledPin, HIGH);
          for(int i =0; i>1000;i++){
            delay(10);
            sendPacket(0,argName,0,0,203,0);    //255 is for broadcast2 and 203 is for shudown
          }  
          sendPacket(0,argName,0,0,203,1); 
          Serial.println("You typed shutdown");
          digitalWrite(ledPin, LOW);
        }
        else if(!strcmp(arg3, "land") && ((argName<NUM_NODES)||(argName==255))){
          digitalWrite(ledPin, HIGH);
          for(int i =0; i>1000;i++){
            delay(10);
            sendPacket(0,argName,0,0,204,0);    //255 is for broadcast2 and 204 is for land 
          }
          sendPacket(0,argName,0,0,204,1); 
          Serial.println("You typed land");
          digitalWrite(ledPin, LOW);
        }
        else if(!strcmp(arg3, "flight") && ((argName<NUM_NODES)||(argName==255))){
          digitalWrite(ledPin, HIGH);
          for(int i =0; i>1000;i++){
            delay(10);
            sendPacket(0,argName,0,0,205,0);    //255 is for broadcast2 and 205 is for flight                        
          } 
          sendPacket(0,argName,0,0,205,1);
          Serial.println("You typed flight");
          digitalWrite(ledPin, LOW);
        }         
      }
      else{
        Serial.print("read in error");
      }


      arg1 = strtok_r(NULL, "\n", &tempCommand1);


      //for(int i = 0; i <= lineCounter; i++){
        //arg1 = strtok(NULL, "\n");
      //}

      //if(arg1 == NULL) break;

      lineCounter++;
      //arg1 = strtok(NULL, "\n");
      //if(arg1 == NULL) break;
    }

    Serial.print("out of while loop\n");


  }
}




/*
      else if(!strcmp(arg1, "reset")){    //might add reset command
 digitalWrite(ledPin, HIGH);
 for(int i =0; i>1000;i++){
 delay(1);
 sendPacket(0,255,0,0,200,0);                            
 } 
 sendPacket(0,255,0,0,200,1);
 Serial.println("You typed reset");
 digitalWrite(ledPin, LOW);
 return 1;
 }*/


/*
//stub have this function passing array and only output status int
 int changeDesiredLocs(int want[][], int curr[][]){
 //This method is where the desired locations of the slave nodes is reset
 //currently, it only adds some randomness to all the points
 //to have the desired locations a little bit somewhere else.
 //(future) Have something more intelligent.
 
 for(int i = 0; i < NUM_NODES; i++){
 desired[i][0] = desired[i][0] + int((rand() % 3) - 2);  //adds -1, 0, or 1
 	    	desired[i][1] = desired[i][1] + int((rand() % 3) - 2);
 }
 return want;
 }
 */

void loop(){
  //This block picks up a new message if the state machine requires one this
  //cycle.  It also accommodates packets not arriving yet
  //It also sets gotNewMsg, which controls data collection later
  if(wantNewMsg){
    //Save old values in case can't pick up a new packet
    for(int i = 0; i < PACKET_LENGTH; i++){
      oldMsg[i] = currMsg[i];
    }

    //Get new values. If no packet available, goodMsg will be null
    goodMsg = listenForPacket(currMsg);

    //Check to see if packet is null. If so, put old values back
    if(goodMsg == 0){
      for(int i = 0; i < PACKET_LENGTH; i++){
        currMsg[i] = oldMsg[i];
      }
      gotNewMsg = false;
    }
    else{
      //..otherwise, the packet is good, and you got a new message successfully
      lastHeardFrom = currMsg[SENDER];
      gotNewMsg = true;
    }
  }


  //State machine controlling what to do with packet
  switch(state){
  case INIT:
    //Serial.println("Init state");
    //Send Startup message, with SENDER == 0, and TARGET == 0;  Sending 10 times arbitrarily
    digitalWrite(ledPin, HIGH);
    for(int i = 0; i < 10; i++){
      sendPacket(0, 0, 0, 0, 0, 0);
    }
    sendPacket(0, 0, 0, 0, 0, 1);

    state = RECEIVE;
    wantNewMsg = true;
    break;

  case RECEIVE:
    //Serial.println("Receive state");
    digitalWrite(ledPin, LOW);

    //If hear from Prev_Prev, and it's a valid message, "start" timeout timer
    if((currMsg[SENDER] == PREV_PREV_NODE || currMsg[SENDER] == PREV_NODE) && gotNewMsg){
      lastTime = millis();
    }

    //stub, this allows the case where prev_prev fails, and the node hasn't heard from prev_prev
    //so as soon as prev node says anything, the timer is checked and is of course active,
    //so we need another timer to see whether you've last heard anything from prev_prev, and a timer
    //for whether you've last heard anything from prev
    if(lastHeardFrom == PREV_NODE || lastHeardFrom == PREV_PREV_NODE){
      currTime = millis() - lastTime;
    }

    //Debug printing
    //Serial.print("l ");
    //Serial.println(lastTime);
    //Serial.print("c ");
    //Serial.println(currTime);

    //Serial.print("Msg from ");
    //Serial.print(currMsg[SENDER]);
    //Serial.print(" end ");
    //Serial.print(currMsg[END_BYTE]);
    //Serial.print(" rssi ");
    //Serial.println(currMsg[RSSI_INDEX]);

    //Serial.println(lastHeardFrom);

    //Prev node has finish sending
    if(currMsg[SENDER] == PREV_NODE && currMsg[END_BYTE] == byte(1) && lastHeardFrom == PREV_NODE){
      state = CALCULATE;
      wantNewMsg = false;
    } 
    else if((lastHeardFrom == PREV_PREV_NODE && currTime > TIMEOUT_PP)||(lastHeardFrom == PREV_NODE && currTime > TIMEOUT_P)){
      //Serial.println("Timeout");
      state = CALCULATE;
      lastTime = millis();
      currTime = 0;
      wantNewMsg = false;
    }

    //Add data from packet to data structure
    distances[currMsg[SENDER]][currMsg[TARGET]] = currMsg[DISTANCE];
    allSensorData[currMsg[SENDER]] = currMsg[SENSOR_DATA];

    break;

  case CALCULATE:
    //Serial.println("Calculate state");
    digitalWrite(ledPin, HIGH);

    //Assert "distance to self"s to 0 then
    //average upper and lower triangles of data
    for(int i = 0; i < NUM_NODES; i++){
      distances[i][i] = 0; 
      int h = i;
      for(h = h + 1; h < NUM_NODES; h++){
        distances[i][h] = roundUp((distances[i][h] + distances[h][i])/2);
        distances[h][i] = distances[i][h];
      }
    }
    /*
    //Transmit data through serial
     roundNumber = roundNumber + 1;
     //Hard-coded formatting that R knows to accept, starting with round number and number of nodes
     Serial.print("-3 ");
     Serial.println(roundNumber);
     Serial.print("-1 0 ");
     Serial.println(NUM_NODES);
     //Send anchor node values
     Serial.print("-1 1 ");
     Serial.println(x1);
     Serial.print("-1 2 ");
     Serial.println(y1);
     Serial.print("-1 3 ");
     Serial.println(x2);
     Serial.print("-1 4 ");
     Serial.println(y2);
     Serial.print("-1 5 ");
     Serial.println(x3);
     Serial.print("-1 6 ");
     Serial.println(y3);
     //Send current distance values
     for(int i = 0; i < NUM_NODES; i++){
     for(int j = 0; j < NUM_NODES; j++){
     Serial.print(i);
     Serial.print(" ");
     Serial.print(j);
     Serial.print(" ");
     Serial.println(distances[i][j]);
     }
     }
     //Send desired values
     for(int i = 0; i < NUM_NODES; i++){
     Serial.print("-2 ");
     Serial.print(i);
     Serial.print(" 0 ");
     Serial.println(desired[i][0]);
     
     Serial.print("-2 ");
     Serial.print(i);
     Serial.print(" 1 ");
     Serial.println(desired[i][1]);
     }
     //Send round number again, as "end of serial transmission" indicator
     Serial.print("-4 ");
     Serial.println(roundNumber);*/

    //Stub, read from Serial to detect results from R, then read in until
    //receive "-4 [roundNumber]" back (signifies end of transmission)
    //For now, just delay for a second
    delay(1000);
    
    userCom();


    //Parse returned calculated values
    //for(int i = 0; i < NUM_NODES; i++){
    //currLoc[i][0] = line[1];
    //currLoc[i][1] = line[2];
    //}

    //Compare current network locations to desired locations, decide to change desired locations
    //boolean closeEnough = true;
    //for(int i = 0; i < NUM_NODES; i++){
    //  if((abs(desired[i][0] - currLoc[i][0]) < 0) && (abs(desired[i][0] - currLoc[i][0]) > 2)) closeEnough = false;
    //  if((abs(desired[i][1] - currLoc[i][1]) < 0) && (abs(desired[i][1] - currLoc[i][1]) > 2)) closeEnough = false;
    //}
    //if(closeEnough){
    //	desired = changeDesiredLocs(desired, currLoc);
    //}

    //Send current locations and commands a few times
    for(int j = 0; j < REDUNDANCY; j++){
      for(int i = 0; i < NUM_NODES; i++){
        sendPacket(MY_NAME, i, currLoc[i][0], currLoc[i][1], 0, 0);
        sendPacket(MY_NAME, i, desired[i][0], desired[i][1], 0, 0); //!!!!!!!!!!!!!!!! which one identifies the type of
      }
    }

    //why not broadcast to all node to end it, since they are all easedropping.
    sendPacket(MY_NAME, NUM_NODES, desired[NUM_NODES][0], desired[NUM_NODES][1], 1, 1); 

    lastHeardFrom = MY_NAME;

    //Turn is over, begin receiving again
    wantNewMsg = true;
    state = RECEIVE;
    break;
  }

  //Every received packet must have RSSI scraped off and added to calculations
  if(gotNewMsg){
    //Check if there is already an average, if so, do filter, if not just add data in appropriate position
    //(in both cases pointer must be incremented or looped
    if(rssiAvg[currMsg[SENDER]] != 0){
      //Filter RSSI based on +-10 around running average
      //if(currMsg[RSSI_INDEX] < rssiAvg[currMsg[SENDER]] + 10 && currMsg[RSSI_INDEX] > rssiAvg[currMsg[SENDER]] - 10){
      rssiData[currMsg[SENDER]][rssiPtr[currMsg[SENDER]]] = currMsg[RSSI_INDEX];
      if(rssiPtr[currMsg[SENDER]] == STRUCT_LENGTH - 1) rssiPtr[currMsg[SENDER]] = 0;  //Loop pointer around if it's reached the end of the array
      else rssiPtr[currMsg[SENDER]] += 1;                              //..otherwise just increment

      //}
    }
    else{
      rssiData[currMsg[SENDER]][rssiPtr[currMsg[SENDER]]] = currMsg[RSSI_INDEX];
      if(rssiPtr[currMsg[SENDER]] == STRUCT_LENGTH - 1) rssiPtr[currMsg[SENDER]] = 0;
      else rssiPtr[currMsg[SENDER]] += 1;
    }


    //If there's a full row of data to average, average it
    //(detects full row by checking last bit in row filled)
    if(rssiData[currMsg[SENDER]][STRUCT_LENGTH - 1] != 0){
      temp = 0;
      for(int i = 0; i < STRUCT_LENGTH; i++){
        temp += rssiData[currMsg[SENDER]][i];
      }
      rssiAvg[currMsg[SENDER]] = temp/STRUCT_LENGTH;
    }

    //Add new average to distance table
    distances[MY_NAME][currMsg[SENDER]] = roundUp(log(float(rssiAvg[currMsg[SENDER]])/95)/log(0.99));

  }

  //userCom();
  
  //Transmit data through serial
     roundNumber = roundNumber + 1;
     //Hard-coded formatting that R knows to accept, starting with round number and number of nodes
     Serial.print("-3 ");
     Serial.println(roundNumber);
     Serial.print("-1 0 ");
     Serial.println(NUM_NODES);
     //Send anchor node values
     Serial.print("-1 1 ");
     Serial.println(x1);
     Serial.print("-1 2 ");
     Serial.println(y1);
     Serial.print("-1 3 ");
     Serial.println(x2);
     Serial.print("-1 4 ");
     Serial.println(y2);
     Serial.print("-1 5 ");
     Serial.println(x3);
     Serial.print("-1 6 ");
     Serial.println(y3);
     //Send current distance values
     for(int i = 0; i < NUM_NODES; i++){
     for(int j = 0; j < NUM_NODES; j++){
     Serial.print(i);
     Serial.print(" ");
     Serial.print(j);
     Serial.print(" ");
     Serial.println(distances[i][j]);
     }
     }
     //Send desired values
     for(int i = 0; i < NUM_NODES; i++){
     Serial.print("-2 ");
     Serial.print(i);
     Serial.print(" 0 ");
     Serial.println(desired[i][0]);
     
     Serial.print("-2 ");
     Serial.print(i);
     Serial.print(" 1 ");
     Serial.println(desired[i][1]);
     }
     //Send round number again, as "end of serial transmission" indicator
     Serial.print("-4 ");
     Serial.println(roundNumber);
     delay(1000);
     
     
     
     
     

  /*stuckUser =0;
   do{
   int mode = userCom();
   if(mode==0){
   stuckUser = 1;
   }
   else if(mode==1){
   stuckUser =0;
   break;
   }
   
   }
   while(stuckUser == 1);
   
   */

  //userCom() outputs
  //no new command is 0
  //shutdown is 1
  //flight 2
  //land is 3
  /*int exit = 1;
   int prevCmd = 0;
   int currCmd;
   while(exit){
   currCmd = userCom();
   if(currCmd != 0){ //a new command has been entered
   if(prevCmd == 1 || currCmd == 1){
   exit = 1;
   }
   else if(currCmd == 2){
   exit = 0;
   }
   else{
   exit = 1;
   }
   prevCmd = currCmd;
   }
   Serial.println(".");
   }*/


}





