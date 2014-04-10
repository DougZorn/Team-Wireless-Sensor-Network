/*  
 *  
 *  Slave Node Code
 *  
 *  Before using, check:
 *  NUM_NODES, MY_NAME, PREV_NODE, PREV_PREV_NODE
 *
 */
#include <SPI.h>
#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"
#include "math.h"

//Number of nodes, including Command Node
const byte NUM_NODES = 4;

//Names of Node and nodes before it
//Determines when this node's turn is
const byte        MY_NAME = 1;
const byte      PREV_NODE = 0;
const byte PREV_PREV_NODE = 3;

//How many data entries to take an average from, arbitrarily set to 15 stub
const int STRUCT_LENGTH = 5;

//State names
const int IDLE_S=0;
const int DECIDE=1;
const int SEND=2;
const int RECEIVE=3;
int state;

//The indexes of where each piece of data is (for readability of code)
const int SENDER = 0;
const int TARGET = 1;
const int DISTANCE = 2;
const int SENSOR_DATA = 3;
const int HOP = 4;
const int END_BYTE = 5;
const int RSSI_INDEX = 6;

const int XCOORD = 2;
const int YCOORD = 3;
const int CMD_TYPE = 6; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! RSSI, use something else

//This is how many times to resend all data, for redundancy.  Arbitrarily set to 4
const int REDUNDANCY = 4; 

//This is how close a node can be to its desired position to merit Not moving
const int NEARTOLERANCE = 10;  //arbitrarily set to 10 inches

//A bunch of globals.
//Timer info
const unsigned long TIMEOUT_PP = 3000; //??? check this timeout number stub
const unsigned long TIMEOUT_P = 1000; 

//Shows whether a new packet has arrived this turn
boolean gotNewMsg;

//Flag for controlling getting new data every cycle or not stub
boolean wantNewMsg;

//Flag for entering movement loop
boolean moveRequired;

//These control timeouts
unsigned long currTime;
unsigned long lastTime;

//These are the structures that contain  data to be averaged of Rssi values
byte rssiData[NUM_NODES][STRUCT_LENGTH];

//Each contains a pointer for each of the nodes, indicating where to write in the above tables
int rssiPtr[NUM_NODES];

//Final array of distances averaged from the rssiData arrays, to be sent out next turn
byte rssiAvg[NUM_NODES];
byte distances[NUM_NODES];

//Coords of this node, current and desired
//stub these will be negative or positive
int currX, currY, desX, desY;

//Dummy value for now, will be filled later by sensor function stub
byte sensorData = 5;



//The current message
byte currMsg[PACKET_LENGTH];
byte oldMsg[PACKET_LENGTH];

//Current RSSI/LQI values, stub what this data type is too
byte currMsgRssi;
byte oldMsgRssi;

//Helps coordinate Timeout
byte lastHeardFrom;

//Temporary variable for calculating averages
unsigned int temp;
int goodMsg;


//Converts values from 0-255 to (-)127-128
int byteToInt(byte input){
  if(input > 128){
    input = input - 128;
    return input;
  }
  else if(input < 128){
    input = input * -1;
    return input;
  }
  else return 0;
}

//STUB  I'm including math.h to use trig functions, and there are round, ceil, and floor functions therein
//Rounds ints and casts to byte 
byte roundUp(float input){
  float output = input + 0.9999;
  return byte(output);
}

void resetData(){
  digitalWrite(9, LOW);
  gotNewMsg = false;
  wantNewMsg = true;
  moveRequired = false;
  state = IDLE_S;
  currTime = 0;
  lastHeardFrom = 255;
  currX = 0;
  currY = 0;
  desX = 0;
  desY = 0;
  goodMsg = 0;
  unsigned long lastTime;
  for(int x = 0; x<NUM_NODES; x++){
    rssiPtr[x] = 0;
    rssiAvg[x] = 0;
    distances[x] = 0;
    for(int y = 0; y<STRUCT_LENGTH; y++){
      rssiData[x][y] = 0;
    }
  }
  for(int x = 0; x<PACKET_LENGTH; x++){
    currMsg[x] = 255;
    oldMsg[x] = 255;
  }
}

void setup(){
  Serial.begin(9600);
  init_CC2500_V2();
  pinMode(9,OUTPUT);
  resetData();
}

void loop(){
  //This block picks up a new message if the state machine requires one this
  //cycle.  It also accommodates packets not arriving yet, and checksum not
  //passing.  It also sets gotNewMsg, which controls data collection later
  if(wantNewMsg){
    //Save old values
    for(int i = 0; i < PACKET_LENGTH; i++){
      oldMsg[i] = currMsg[i];
    }

    //Get new values, if no packet available, currMsg will be null
    goodMsg = listenForPacket(currMsg);

    //Check to see if packet is null, or checksum is bad.  If so, put old values back
    if(goodMsg == 0){
      for(int i = 0; i < PACKET_LENGTH; i++){
        currMsg[i] = oldMsg[i];
      }
      gotNewMsg = false;
      //Serial.println("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
    }
    else{
      //..otherwise, the packet is good, and you got a new message successfully
      gotNewMsg = true;
      lastHeardFrom = currMsg[SENDER];
      Serial.print("Msg from ");
      Serial.print(currMsg[SENDER]);
      Serial.print(" to ");
      Serial.print(currMsg[TARGET]);
      Serial.print(" end ");
      Serial.println(currMsg[END_BYTE]);
    }
  }

  //State machine controlling what to do with packet
  switch(state){

    //Idle case is directly to wait until the command node sends startup message,
    //then is never used again
  case IDLE_S: 
    Serial.println("IDLE");
    digitalWrite(9, LOW);

    //Serial.println("Idle State");
    //check receive FIFO for Startup Message
    if(currMsg[SENDER] == 0 && currMsg[TARGET] == 0){  //"Command is sender and target is 0" is the startup message
      state = DECIDE;
      //resetData();
    }
    wantNewMsg = true;
    break;

    //Decide case is just to control whether to go to RECEIVE or SEND
  case DECIDE:
    //Serial.println("DECIDE");
    digitalWrite(9, LOW);
    //Serial.println("Decide State");

    //if you hear something from prev_prev, and it's actually a good message, reset the timer
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

    //This node's turn occurs if either the previous node has sent its final message
    //or if the timeout has occurred since the previous previous node has sent its final message
    //if(currMsg[SENDER] == 0 && currMsg[TARGET] == 0 && currMsg[DISTANCE] == 0 && currMsg[SENSOR_DATA] == 0 && currMsg[HOP] == 0){  //"Command is sender and target is 0" is the startup message
      //state = IDLE_S;
      //wantNewMsg = false;
    //}else 
    if(currMsg[SENDER] == PREV_NODE && currMsg[END_BYTE] == byte(1) && lastHeardFrom == PREV_NODE){
      state = SEND;
      wantNewMsg = false;
      lastTime = millis();
      currTime = 0;
     // Serial.println("a");
    }else if((lastHeardFrom == PREV_PREV_NODE && currTime > TIMEOUT_PP)||(lastHeardFrom == PREV_NODE && currTime > TIMEOUT_P)){
      state = SEND;
      lastTime = millis();
      currTime = 0;
      wantNewMsg = false;
     // Serial.println("b");
    }
    else if(gotNewMsg == false){ //If you didn't successfully get a packet this iteration, just stay in DECIDE and try again
      state = DECIDE;
      wantNewMsg = true;
     // Serial.println("c");
    }
    else if(currMsg[TARGET]==MY_NAME){ //..otherwise just go to RECEIVE to handle cases next time, and don't pick up a new packet
      state = RECEIVE;
      wantNewMsg = false;
    }

    break;

    //SEND case is either to send all data that you have, some number of times, with the last message having END bit high
    //or to send a bunch of null packets, with the last message having END bit high
  case SEND:
    //Serial.println("SEND");
   // Serial.println("Send State");
    digitalWrite(9, HIGH);

    lastHeardFrom = MY_NAME;
    //If you've got at least the first four node's data, send everything you have
    //This is a reasonable assumption because the network needs three known points anyway
    //so we can trust that nodes 0, 1, 2, 3 will very probably exist;
    int i;
    if(distances[PREV_NODE] != 0 && distances[PREV_PREV_NODE] != 0){
      //for(int j = 0; j < REDUNDANCY; j++){  //stub, resend all data some number of times for redundancy?
      //Serial.print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      for(int i = 0; i < NUM_NODES; i++){
        //if you have the data for a node, send it
        if(distances[i] != 0) sendPacket(MY_NAME, i, distances[i], sensorData, 0, 0);
        //Serial.print("Value for ");
       // Serial.print(i,DEC);
        //Serial.print(": ");
        //Serial.println(distances[i]);
      }
      //}

      //..then send final packet with END set high
      sendPacket(MY_NAME, i, distances[NUM_NODES], sensorData, 0, 1);

    }
    else{ //Not enough packets, send as many nulls as other nodes need to get an average from this node
      for(int i = 0; i < STRUCT_LENGTH; i++){
        sendPacket(MY_NAME, MY_NAME, 0, 0, 0, 0);//stub byte conversion with neg values
      }

      //..also send final packet with END set high
      sendPacket(MY_NAME, MY_NAME, 0, 0, 0, 1);//stub byte conversion with neg values
    }

    //Return to DECIDE and try to get new packet
    state = DECIDE;
    wantNewMsg = true;
    break;

    //RECEIVE state just control some special conditions that need to be looked for and caught,
    //specifically commands and timeout (additional timeout for prev_prev_prev_node can be added easily here)
  case RECEIVE:
    Serial.println("RECEIVE");
    Serial.println(" ");
    //Serial.println("Receive State");
    digitalWrite(9, LOW);
    if(currMsg[SENDER] == 0 && currMsg[CMD_TYPE] == 0){  //message contains this node's current position
      currX = byteToInt(currMsg[XCOORD]);
      currY = byteToInt(currMsg[YCOORD]);
    }
    else if(currMsg[SENDER] == 0 && currMsg[CMD_TYPE] == 1){  //message contains this node's desired position
      desX = byteToInt(currMsg[XCOORD]);
      desY = byteToInt(currMsg[YCOORD]);
      //stub, this is where movement variables are checked and changed (actual movement to be handled below)
      
      if(abs(currX - desX) > NEARTOLERANCE || abs(currY - desY) > NEARTOLERANCE){
        moveRequired = true;
      }
    }
    state = DECIDE;
    wantNewMsg = true;
    gotNewMsg = false;
    break;
  }

  //Serial.println("++++++++++++++++++++++++++");
  //Every cycle there is a new packet in currMsg, do RSSI/LQI
  //averaging, choosing, and conversion
  if(gotNewMsg){
    //Filter RSSI based on +-10 around running average, if there is one
    //make sure pointer is in right spot, then put new data in that location
    if(rssiAvg[currMsg[SENDER]] != 0){//stub changed bracket in merge
      //rssi filtering with adjustable range of values 
      //allowed needs running average to work, so this disables filtering until 
      //an average is reached.
      //if(currMsg[RSSI_INDEX] < rssiAvg[currMsg[SENDER]] + 10 && currMsg[RSSI_INDEX] > rssiAvg[currMsg[SENDER]] - 10){
        rssiData[currMsg[SENDER]][rssiPtr[currMsg[SENDER]]] = currMsg[RSSI_INDEX];
        if(rssiPtr[currMsg[SENDER]] == STRUCT_LENGTH - 1) rssiPtr[currMsg[SENDER]] = 0;  //Loop pointer around if it's reached the end of the array
        else rssiPtr[currMsg[SENDER]] += 1;                          //..otherwise just increment
      //}
    }
    else{
      rssiData[currMsg[SENDER]][rssiPtr[currMsg[SENDER]]] = currMsg[RSSI_INDEX];
      if(rssiPtr[currMsg[SENDER]] == STRUCT_LENGTH - 1) rssiPtr[currMsg[SENDER]] = 0;
      else rssiPtr[currMsg[SENDER]] += 1;
    }

    //Detects full row of data to average by checking if last bit 
    //in row is filled, then averages the row
    if(rssiData[currMsg[SENDER]][STRUCT_LENGTH - 1] != 0){
      temp = 0;
      Serial.print("RSSI: ");
      for(int i = 0; i < STRUCT_LENGTH; i++){
        temp += rssiData[currMsg[SENDER]][i];
        Serial.print(rssiData[currMsg[SENDER]][i], DEC);
        Serial.print(" ");
      }
      Serial.print("TEMP: ");
      Serial.println(temp);
      rssiAvg[currMsg[SENDER]] = temp/STRUCT_LENGTH;
      Serial.print("RSSI AVG: ");
      Serial.println(rssiAvg[currMsg[SENDER]], DEC);
    }else{
      Serial.println("RSSI Array Not Full");
    }

    //Calculate distance from RSSI values and add to distance array
    distances[currMsg[SENDER]] = roundUp(log(float(rssiAvg[currMsg[SENDER]])/95)/log(0.99));
    //Serial.print("Distance: ");
    //Serial.println(distance[currMsg[SENDER]],DEC);
    //Serial.println(" ");
    if((currMsg[SENDER] == 0 )&& (currMsg[TARGET] == 0) && (currMsg[DISTANCE] == 0) && (currMsg[SENSOR_DATA] == 0) && (currMsg[HOP] == 200)){
        resetData();
      /*Serial.print("Sender:");
      Serial.println(currMsg[SENDER]);
      Serial.print("Target: ");
      Serial.println(currMsg[TARGET]);
      Serial.print("Distance: ");
      Serial.println(currMsg[DISTANCE]);
      Serial.print("Data: ");
      Serial.println(currMsg[SENSOR_DATA]);
      Serial.print("Hop: ");
      Serial.println(currMsg[HOP]);*/
        Serial.println("RESETED");
      }
    //Serial.println(" ");
    //gotNewMsg = false;
  }
  

  //Code for sending a movement to the motors will go here, 
  //in RECEIVE state there will be flags and variables set to control movement
  //In this block there will be signals sent to motors, and various counters
  //so that each movement signal will only occur for so many milliseconds/etc
  //to ensure that a little bit of movement happens every cycle, and the
  //movement itself is not blocking the rest of the code
  //This block will also check to see if any distance to another node is
  //too close, and will either just not move, or (future) try to move around
  //the obstacle inme rudimentary way so
  
  //Movement is comprised of Lifting off, landing, or hovering/moving
  
  //Close to ground, and you haven't set flag, so take off
  //you set flag so you can land again when session is over, and so
  //you don't immediately take off as soon as you land
  if(sonic < ---- && !flag){
    liftoff();
  }else{
    //set a flag to say you have taken off already
    //so you don't get stuck unable to land again
  }
  
  //When landing flag is raised, decrease power if you're above some height
  //until you get to some other height determined by the sonic (close to something
  //that's under copter, not necessarily overall height), at which point you 
  //set throttle to some specific, slow descending amount.
  if(landflag){
    if(sonic < ----){
      //set throttle = some low number to come to final spot
      //otherwise, start descending if you're above some altitude
    }else if(alt > ----){
      //throttle down a bit to start descending
    }else if(alt > ---- && alt < ----){
      //do nothing to slowly descend
    }else {  }
    
    //check to see if you're moving too fast
    if(imuMagnitude > ----){
      //so Throttle up a bit. 
    }
  }
  
  if(moveRequired){
    //alt - altitude
    //sonic - ultrasonic
    
    int mag;
    //mag = pickUpMagData();
    //Assuming the mag data is some number of degrees off of due north, +-180, with +180 going counter clockwise
    
    //Calculate angle we need to go
    int dX = desX - currX;
    int dY = desY - currY;
    int angle = int(round(atan(double(abs(dY)/abs(dX)))));
    
    //consider offset from north, taking into account abs of atan (see Erik's notes)
    if(dX < 0 && dY > 0){
      angle = 90 - angle;
    }else if(dX > 0 && dY > 0){
      angle = -1*(90 - angle);
    }else if(dX < 0 && dY < 0){
      angle = 90 + angle;
    }else if(dX > 0 && dY < 0){
      angle = -1*(90 + angle);
    }
    
    int dist = int(sqrt(pow(double(dX), 2) + pow(double(dY), 2)));
    
    //Spin or Move
    //if you're more than 15 degrees off
    if(abs(mag - angle) > 15){
      if((mag - angle) <= 0){
        //spin clockwise (when looking down on copter)
      }else{
        //spin counter clockwise
      }
    }else if(dist > NEARTOLERANCE){
      //pitch forward
    }
    
    
    
    moveRequired = false; 
  }

}

