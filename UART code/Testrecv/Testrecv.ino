/*  
 *  
 *  Slave Node Code
 *  
 *  Before using, check:
 *  NUM_NODES, MY_NAME, PREV_NODE, PREV_PREV_NODE
 *
 */
#include <SoftwareSerial.h>      //needed for SoftwareSerial


//Declare Pins for UART
SoftwareSerial mySerial(8, 7);   // RX, TX
byte uartArray[64] = {0};
int curSpot;
int prtSpot;
int upDated;

typedef struct {                    //array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            //smoother version of accADC
  int16_t  gyroData[3];             //Not sure
  int16_t  magADC[3];              //180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             //raw gyro data
  int16_t  accADC[3];              //raw accelerometer data
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
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


typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt_t;

alt_t alt;*/
/*  //might need it
typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;
*/

void initData(){
  myData.EstAlt =0;
  myData.vario =0;
  myData.ultraSonic =0;
  oldData.EstAlt =0;
  oldData.vario =0;
  oldData.ultraSonic =0;
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

void backupData(){
  oldData.magADC[0]= myData.magADC[0];
  oldData.magADC[1]= myData.magADC[1];
  oldData.magADC[2]= myData.magADC[2];
  
  /* // add as we need them
  oldData.accSmooth[0]= myData.accSmooth[0];
  oldData.accSmooth[1]= myData.accSmooth[1];
  oldData.accSmooth[2]= myData.accSmooth[2];
  */
  
  oldData.EstAlt = myData.EstAlt; 
}
void revertData(){
  myData.magADC[0] = oldData.magADC[0]; 
  myData.magADC[1] = oldData.magADC[1];
  myData.magADC[2] = oldData.magADC[2];
  
  /* // add as we need them
  */
  
  myData.EstAlt = oldData.EstAlt; 
}
int storeData16(int dType, int16_t data16){
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
    default :
      return 1;
  }
  return 0;
}

int storeData32(int dType, int32_t data32){
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

void writeData16(int16_t data){
  byte temp;
  temp = data>>8;  
  mySerial.write(temp);
  temp = data;
  mySerial.write(temp);    
}

int updateData(byte *array){
  if(upDated !=1){
    return 1; 
  }
  byte sensorType;
  byte startByte = 0x80;
  byte endByte  = 0xC0; 
  int16_t tempData16;                   
  int32_t tempData32;
  //int flag = 0;
  int place = 0;
  
  backupData();
  
  while((array[place] != startByte)&&(place <64)){
    place++;
    //Serial.print("here: ");
    //Serial.println(place);
  } 
  
  if(place>=64){
    return 1;
  }
  place++;
  
  do{
    if((sensorType = array[place]) < 32){
      place++;
      tempData16 = array[place];
      
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
      storeData16(sensorType, tempData16);
    }else{
      place++; 
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
      storeData32(sensorType, tempData32);
    }
     
    place++;
    if(place>=64){
      revertData();
      return 1;  
    }
  }while(array[place]!=endByte);
  
  for(int CT; CT<64;CT++){
    array[CT] = 0;
  }
  mySerial.flush();
  return 0;
  
}



void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(9,OUTPUT);
}

void loop(){
  curSpot=0;
  prtSpot = 0;
  upDated = 0;
  
  
  
  while(mySerial.available()){ //maybe add || certain byte: hardcoded.
    delay(1);    //millis here to avoid missed chained of bytes, dynamic code too restrictive 
    uartArray[curSpot++] = mySerial.read();
    upDated=1;
    if(curSpot>=64){
      mySerial.flush();
      break; 
    }
  }
  while((prtSpot < curSpot)&& upDated){
    Serial.print(uartArray[prtSpot], HEX);
    Serial.print(" ");
    prtSpot++;
  } 
  
  if(upDated==1){
    Serial.println(" ");
  }
  /*
  if(updateData(uartArray)==0){
    //updateData(uartArray);
    Serial.print(myData.magADC[0],HEX);
    Serial.print(" ");
    Serial.print(myData.magADC[1],HEX);
    Serial.print(" ");
    Serial.print(myData.magADC[2],HEX);
    Serial.print(" ");
    Serial.print(myData.EstAlt,HEX);
    Serial.println(" ");
  }*/
  
}



