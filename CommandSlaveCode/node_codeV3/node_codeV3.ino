/*  
 *  
 *  Slave Node Code
 *  
 *  Before using, check:
 *  NUM_NODES, MY_NAME, PREV_NODE, PREV_PREV_NODE
 *
 */
#include <SPI.h>
#include <string.h>
#include <SoftwareSerial.h>      //needed for SoftwareSerial
#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"
#include "motorcontrol.h"

//Declare Pins for UART
SoftwareSerial mySerial(8, 7);   // RX, TX


//The LED PIN
int ledPin = 9;


//At the right height level
boolean atLevel;

//Number of nodes, including Command Node
const byte NUM_NODES = 4;

//Names of Node and nodes before it
//Determines when this node's turn is
const byte        MY_NAME = 1;
const byte      PREV_NODE = 0;
const byte PREV_PREV_NODE = 3;

//flag for checking if RSSI array is full
boolean RSSIArrayFull;

//flag for being in air
int Flight;

//the distance between desire and current location before actually changing in INCHES
const int NEARTOLERANCE = 5;

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
const int CMD_TYPE = 4;

//This is how many times to resend all data, for redundancy.  Arbitrarily set to 4
const int REDUNDANCY = 4; 


//A bunch of globals.
//Timer info
const unsigned long TIMEOUT_PP = 30; //??? check this timeout number stub
const unsigned long TIMEOUT_P = 10; 

//Shows whether a new packet has arrived this turn
boolean gotNewMsg;

//Flag for controlling getting new data every cycle or not stub
boolean wantNewMsg;


//Flag for if movement is enabled
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

//UltraSonic Stuff
int UltraSonicPin = A0;    // select the input pin for the potentiometer

//Uart Variables
byte uartArray[64];
int curSpot;
int prtSpot;
int upDated;

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

//Rounds ints and casts to byte 
byte roundUp(float input){
  float output = input + 0.9999;
  return byte(output);
}

//structure for holding sensor information
typedef struct {                    //array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            //smoother version of accADC
  int16_t  gyroData[3];             //Not sure
  int16_t  magADC[3];              //180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             //raw gyro data
  int16_t  accADC[3];              //raw accelerometer data
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
  int16_t  heading;
  uint32_t ultraSonic;        //in cm
} data_t;

data_t myData;

//used for backup incase of screwup
data_t oldData;

/*  //might need it
typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;
} analog_t;
*/
/*  //might need it
typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;
*/

void initData(){              //initialize everything in the backup and current UART sensor datas
  myData.EstAlt =0;
  myData.vario =0;
  myData.ultraSonic =0;
  oldData.EstAlt =0;
  oldData.vario =0;
  oldData.ultraSonic =0;
  myData.heading =0;
  oldData.heading =0;
  for(int x = 0; x<3;x++){
    myData.accSmooth[x]=0;            //smoother version of accADC
    myData.gyroData[x]=0;             //Not sure
    myData.magADC[x]=0;              //180deg = 180, -180deg = -180
    myData.gyroADC[x]=0;             //raw gyro data
    myData.accADC[x]=0;
    oldData.accSmooth[x]=0;            //smoother version of accADC
    oldData.gyroData[x]=0;             //Not sure
    oldData.magADC[x]=0;              //180deg = 180, -180deg = -180
    oldData.gyroADC[x]=0;             //raw gyro data
    oldData.accADC[x]=0;
  }
  for(int CT; CT<64;CT++){
    uartArray[CT] = 0;
  }
}

void backupData(){            //back up old data incase update failed
  oldData.magADC[0]= myData.magADC[0];
  oldData.magADC[1]= myData.magADC[1];
  oldData.magADC[2]= myData.magADC[2];
  
  /* // add as we need them
  oldData.accSmooth[0]= myData.accSmooth[0];
  oldData.accSmooth[1]= myData.accSmooth[1];
  oldData.accSmooth[2]= myData.accSmooth[2];
  */
  oldData.heading = myData.heading; 

  oldData.EstAlt = myData.EstAlt; 
}

void revertData(){  //put old data back to current data, used when update failed
  myData.magADC[0] = oldData.magADC[0]; 
  myData.magADC[1] = oldData.magADC[1];
  myData.magADC[2] = oldData.magADC[2];
  
  /* // add as we need them
  */
  myData.heading = oldData.heading; 
  myData.EstAlt = oldData.EstAlt; 
}

int storeData16(int dType, int16_t data16){//determine what type of int 16 data it is, and store it in correct place, will add as we go along
  switch(dType){
    case 1: 
      myData.magADC[0] = data16;
      break;
    case 2:
      myData.magADC[1] = data16;
      break;
    case 3:
      myData.magADC[2] = data16;
      break;
    case 4:
      myData.heading = data16;
    default :
      return 1;
  }
  return 0;
}

int storeData32(int dType, int32_t data32){  //determine what type of int 32 data it is, and store it in correct place, will add as we go along
  switch(dType){
    case 32:
      myData.EstAlt = data32;
      break;
    case 3:
      Serial.println("case 3");
    break;
    default :
      return 1;
  }
  return 0;
}

void writeData16(int16_t data){  //used to write to uart, if our data is int16
  byte temp;
  temp = data>>8;  
  mySerial.write(temp);
  temp = data;
  mySerial.write(temp);    
}

int updateData(byte *array){  //Ultra Sonic will still update even if uart does not update data when calling this

  myData.ultraSonic = analogRead(UltraSonicPin);    // update ultraSonic data no matter what, it does not use uart from control board

  if(upDated !=1){          //see if update from uart
    return 1;               
  }
  byte sensorType;
  byte startByte = 0x80;    //Byte indicating the Start of chain of packets
  byte endByte  = 0xC0;     //Byte indicating the End of chain of packets
  int16_t tempData16;                   
  int32_t tempData32;
  //int flag = 0;
  int place = 0;            //Locate where packet in array starts, eliminates garbage in front of start if any
  
  backupData();            //puts current data into another same structure, just in case we need them
  
  while((array[place] != startByte)&&(place <64)){    //locate where start of packet is
    place++;
    //Serial.print("here: ");
    //Serial.println(place);
  } 
  
  if(place>=64){      //If not start of packet is found, return fail
    return 1;
  }
  place++;            //move to next byte after start packet
  
  do{
    if((sensorType = array[place]) < 32){        //look at the type of data in the packet, there are usually multiple different types, all type < 32 are or int16 sensors
      place++;                                   // move there
      tempData16 = array[place];                //assemble them because they are int16 or int 32 and uart only sents bytes
      
      //Serial.print(" Data_x1: ");
      //Serial.print(tempData16, HEX);
      tempData16 = tempData16 <<8;
      //Serial.print(" Data_x2: ");
      //Serial.print(tempData32, HEX);
      
      place++;
      tempData16 += array[place];;
      //Serial.print(" Type: ");
      //Serial.print(sensorType, HEX);
      //Serial.print(" Data: ");
      //Serial.println(tempData16, HEX);
      storeData16(sensorType, tempData16);    //store the assembled data into cor
    }else{                                    //if int32 sensors
      place++;                               //assemble
      tempData32 = array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      tempData32 = tempData32 << 8;
      place++;
      
      tempData32 += array[place];
      //Serial.print("Type: ");
      //Serial.print(sensorType, HEX);
      //Serial.print(" Data: ");
      //Serial.println(tempData32, HEX);
      storeData32(sensorType, tempData32);    //store it in correct place
    }
     
    place++;
    if(place>=64){    //if it reach the max buffer size and array size without seeing End Byte, it is cuted off
      revertData();  //put old data back and return fail
      return 1;  
    }
  }while(array[place]!=endByte);  //loop until seeing endByte
  
  for(int CT; CT<64;CT++){      //reset the array for storing data
    array[CT] = 0;
  }
  mySerial.flush();            //flush everything left in the uart rx buffer, size 64 bytes
  
  return 0;
  
}


void resetData(){    //used to initialize data and reset when reseting all nodes, puts all data to default values
  digitalWrite(ledPin, LOW);
  initData();
  gotNewMsg = false;
  RSSIArrayFull = false;
  wantNewMsg = true;
  state = IDLE_S;
  currTime = 0;
  lastHeardFrom = 255;
  currX = 0;
  currY = 0;
  desX = 0;
  desY = 0;
  goodMsg = 0;
  moveRequired = false;
  Flight = 0;
  atLevel =false;
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
  mySerial.begin(9600);
  init_CC2500_V2();
  //initializePWMs();
  pinMode(ledPin,OUTPUT);
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
    digitalWrite(ledPin, LOW);

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
    digitalWrite(ledPin, LOW);
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
    digitalWrite(ledPin, HIGH);

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
        sendPacket(MY_NAME, 255, 0, 0, 0, 0);//stub byte conversion with neg values
      }

      //..also send final packet with END set high
      sendPacket(MY_NAME, 255, 0, 0, 0, 1);//stub byte conversion with neg values
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
    digitalWrite(ledPin, LOW);
    if(currMsg[SENDER] == 0 && currMsg[CMD_TYPE] == 201){  //message contains this node's current position
      currX = byteToInt(currMsg[XCOORD]);
      currY = byteToInt(currMsg[YCOORD]);
    }
    else if(currMsg[SENDER] == 0 && currMsg[CMD_TYPE] == 202){  //message contains this node's desired position
      desX = byteToInt(currMsg[XCOORD]);
      desY = byteToInt(currMsg[YCOORD]);
      //stub, this is where movement variables are checked and changed (actual movement to be handled below)
    }
    
    //Best place for calculating if movement needed
    
    //if in x or y direct, it is off by 3 inches on any side, move to desired location
    //if(abs(currX - desX) > NEARTOLERANCE || abs(currY - desY) > NEARTOLERANCE){
      moveRequired = true;  //more of like we got the desired and current location
    //}
    
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
      RSSIArrayFull = true;
    }else{
      Serial.println("RSSI Array Not Full");
      RSSIArrayFull = false;
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

  //delay(10); stub, not sure if we need a delay, but delays always 
  //made my processing sketches work better way back when        

  //delay(10);


  //+++++++++++++++++++++++++++++++++++++ Update Sensor +++++++++++++++++++++
  //variables for updating data
  curSpot=0;
  prtSpot = 0;
  upDated = 0;  //flag for update
  
  while(mySerial.available()){ //maybe add || certain byte: hardcoded.
    delayMicroseconds(5);    //millis here to avoid missed chained of bytes, dynamic code too restrictive 
    uartArray[curSpot++] = mySerial.read();
    upDated=1;
    if(curSpot>=64){
      mySerial.flush();
      break; 
    }
  }
  
  /* //debugg code
  while((prtSpot < curSpot)&& upDated){
    Serial.print(uartArray[prtSpot], HEX);
    Serial.print(" ");
    prtSpot++;
  } 
  
  if(upDated==1){
    Serial.println(" ");
  }
  
  if(updateData(uartArray)==0){
    //updateData(uartArray);
    Serial.print(myData.magADC[0],DEC);
    Serial.print(" ");
    Serial.print(myData.magADC[1],DEC);
    Serial.print(" ");
    Serial.print(myData.magADC[2],DEC);
    Serial.print(" ");
    Serial.print(myData.EstAlt,DEC);
    Serial.println(" ");
  }*/
  
  //+++++++++++++++++++++++++++++++ Movement ProtoCol +++++++++++++++++++++++++++++
  
  //should be after update for latest info on sensors and adjust the movement, dont go in if doesn't have location,
  // enough data for location, or at the properheight
  if((updateData(uartArray)==0) && moveRequired && RSSIArrayFull&&atLevel){          //update the Data and return if success, Ultrasonic will update no matter what
    
    writeRudder(0x0A);  //set netrual because random 
    writeRoll(0x0A);
    writePitch(0x0A);
    
    delayMicroseconds(100);
    
    byte turn;
    /*
    desX = 75;
    desY = 30;
    currX = 90;
    currY = 10;
    */
    double dX = desX - currX;
    double dY = desY - currY;
    
    //Note: abs rounds the stuff and atan returns radian
    //radian = degree *pie/180
    double angle = atan(dY/dX);  // give -90 to 90
    angle = abs((180*angle)/3.14159265359);
    
    //Serial.println(" ");
    //Serial.println("Distance Angle Test:");
    //Serial.print("angle: ");
    //Serial.println(angle, DEC);
    
    
    if(dX < 0 && dY > 0){        //desX is more on the left than currX 
      angle = -1*(90 - angle);  
    }else if(dX > 0 && dY > 0){
      angle = (90 - angle);
    }else if(dX < 0 && dY < 0){
      angle = -1*(90 + angle);
    }else if(dX > 0 && dY < 0){
      angle = (90 + angle);
    }
    
    int dist = int(sqrt(pow(double(dX), 2) + pow(double(dY), 2)));
    //how often to come in here, because it seems like the movements is going to be 
    //confused if mechanical delay is longer than time of coming into here
    /*
    Serial.print("desX: ");
    Serial.println(desX, DEC);
    Serial.print("desY: ");
    Serial.println(desY, DEC);
    Serial.print("currX: ");
    Serial.println(currX, DEC);
    Serial.print("currY: ");
    Serial.println(currY, DEC);
    Serial.print("dX: ");
    Serial.println(dX, DEC);
    Serial.print("dY: ");
    Serial.println(dY, DEC);
    Serial.print("angle: ");
    Serial.println(angle, DEC);
    Serial.print("dist: ");
    Serial.println(dist, DEC);*/
    
    
    
    
    //Spin or Move
    //if you're more than 15 degrees off
    
    
    
    if(abs(myData.heading - angle) > 15){    //check to see which mag we want and adjust the statment
      
      //Serial.print("inside >15 magADC[0]: ");
      //Serial.println(myData.magADC[0], DEC);
    
      if((myData.heading - angle) <= 0){    
        //spin clockwise (when looking down on copter)
        
        // might want a function to determine how fast to spin
        turn = 0x0A + byte(roundUp((float(abs(myData.heading - angle)/180))*3));  //a function that selects 10 
        writeRudder(turn);
        //Serial.print("turn:  <= 0    => ");
        //Serial.println(turn, HEX);
        
      }else{
        //spin counter clockwise
        turn = 0x0A - byte(roundUp((float(abs(myData.heading - angle)/180))*3));
        writeRudder(turn);
        
        //Serial.print("turn: else   => ");
        //Serial.println(turn, HEX);
      }
    }else if(dist > NEARTOLERANCE){
      
      //Serial.println("in moving dist > NEARTOLERANCE ");
      
      boolean tooClose = false;
      
      //Serial.print("Distance: ");
      for(int i = 0; i< NUM_NODES; i++){
        //Serial.print(distances[i],DEC);
        //Serial.print(" ");
        if(distances[i] < 10){    //if there is a copter within 2 feet to the copter, than it is too close
          if(i != MY_NAME){      //if not myself
            tooClose = true;
            break;
          }
        }  
      }
      
        //Serial.println(" ");
      if(tooClose){
        
        //Serial.println("too close");
        writePitch(0x0A);    //if too close, stay on where it is
      }else{
        
        //Serial.println("far");
        writePitch(0x0C);    //if not, continue moving forward
      }
      
      //here would be where you'd check to see if other nodes are too close
      //for loop through distance values, if any are too low, don't move, else move
      //this would cause a condition though where nodes would get near each other, then
      //both of them would freeze and never move again.  To fix this, we'd need some method
      //of keeping track of if a node is in front of this node (as in, if, when you move forward,
      //the distance to that particular node gets smaller, then it's in front of you and you should
      //stop until that node gets farther away)
      //
      //this would cause another condition where if two nodes are heading directly toward each other
      //they'd stop and freeze, but we'd just need to have some kind of timer to see if a quad
      //has been waiting for another quad to get out of the way for too long, and then just attempt to either
      //roll a bit then move forward, or spin some random amount, then move forward to see if the obstruction
      //is no longer in the way
      
      //the above aside, pitch forward
    }
  }
  
  
  /***************************** LEVEL Handling *******************************************/
  if(Flight==0){            //Flight is 0 when we are flying, 1 when we want to land, 2 when we have landed
  
    atLevel =false;
    //Serial.println("in flight");
    byte heightLevel = 0x0A;
    
    myData.ultraSonic = 195;

    int minHeight = 150; //in cm
    int maxHeight = 160;     
    
    
    if(myData.ultraSonic < 182){   //in cm, this is 6 feet
      
      heightLevel = 0x0A + byte(roundUp(((float(minHeight - myData.ultraSonic)/minHeight)*3)));       //power of moters might change because we added weights
      writeThrust(heightLevel);
      //Serial.print("Too low: ");
      //Serial.print(heightLevel,DEC);
    }else if(myData.ultraSonic > 190){                                             //lower if too above 182 - 190 cm rangle
      heightLevel = 0x0A - byte(roundUp(((float(myData.ultraSonic -maxHeight)/maxHeight)*2))); 
      writeThrust(heightLevel);
      //Serial.print("Too High: ");
      //Serial.print(heightLevel,DEC);
    }else{
      
      //Serial.print("Just Right: ");
      //Serial.print(heightLevel,DEC);
      
      writeThrust(heightLevel);        //keep current level
      atLevel =true;
    }
  }else if(Flight == 1){
    if(myData.ultraSonic <= 21){    //if it is close to ground
      if(myData.EstAlt <5){        //it is right at top of ground
        writeThrust(0x07);          //turns off
        Flight = 2;                //set it so it will ignore flight and land
      }else{                       //if close but not above ground
        writeThrust(0x09);         //slowly go down because it is close to ground
      }
    }else{                         //if not even close to ground
      writeThrust(0x08);           //descend morderately
    }
  }
  
}

