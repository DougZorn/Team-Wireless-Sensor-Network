
/*
Data Structure of Sensor Data and other stuff

typedef struct {                    array[3] because x,y,z or 1,2,3 or Roll, Pitch, Yaw
  int16_t  accSmooth[3];            smoother version of accADC
  int16_t  gyroData[3];             Not sure
  int16_t  magADC[3];              180deg = 180, -180deg = -180
  int16_t  gyroADC[3];             raw gyro data
  int16_t  accADC[3];              raw accelerometer data
} imu_t;

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

typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;
*/

//Note:  Do Not Spam the RX buffere or TX buffer on the Mini or the Control Board
//       it will generate error. 

// SerialAvailable(port number) The port we use is 1. this return the noumber of avaible bytes in the RX buffer
//  EX. uint8_t x = SerialAvailable(1); , if(SerialAvailable(1)>0)

// SerialRead(port number) returns one byte of data from the RX Buffer, uint8_t
// EX. uint8_t y = SerialRead(1);

//SerialUsedTXBuff(port number) returns the number of bytes stored in TX buffer, leave 50byte of space to avoid error

//SerialWrite(port number, one byte of Data) Writes one byte of data to TX buffer, flag TX ISR, Some time later, TX ISR will transmitt all in buffer

void setup() {
  // Open serial communications and wait for port to open:
  //300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
  // set the data rate for the SoftwareSerial port, max at 57600
  // Make sure it is in sync with the Arduino baud rate
  SerialOpen(1,57600);
}

void loop() {
  
  //Example of sending gyro Data
  if(SerialUsedTXBuff(1)>(TX_BUFFER_SIZE - 50)){  //NOTE: Leave at least 50Byte margin to avoid errors
      temp = imu.gyroData[2] >> 8;
      SerialWrite(1,temp);                        //Use SerialWrite will sent data in buffer and hit the flag, then after some time, 
                                                  //ISR will send everything in buffer.  %%%SerialWrite(Port #, uint8_t data)
      temp = imu.gyroData[2] & 0x00FF;
      SerialWrite(1,temp);
  }
}
