#include <SPI.h>
#include "read_write.h"
#include "cc2500_VAL_V2.h"
#include "cc2500_REG_V2.h"

#ifndef INIT_CC2500_V2_H
#define INIT_CC2500_V2_H

#define CC2500_SRES    0x30  

char PATABLE[] = {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void init_CC2500_V2(){        
        SPI.setClockDivider(SPI_CLOCK_DIV2);
        SPI.setDataMode(SPI_MODE0);
        pinMode(SS,OUTPUT);
        SPI.begin();
        digitalWrite(SS,HIGH);  
        SendStrobe(CC2500_SRES);
	WriteReg(REG_IOCFG2,VAL_IOCFG2);                //gdo2output pin configuration 
	WriteReg(REG_IOCFG1,VAL_IOCFG1);                //gdo1output pin configuration 
	WriteReg(REG_IOCFG0,VAL_IOCFG0);                //gdo0output pin configuration 
	WriteReg(REG_FIFOTHR,VAL_FIFOTHR);              //rx fifo and tx fifo thresholds
	WriteReg(REG_SYNC1,VAL_SYNC1);                  //sync word, high byte 
	WriteReg(REG_SYNC0,VAL_SYNC0);                  //sync word, low byte 
	WriteReg(REG_PKTLEN,VAL_PKTLEN);                //packet length 
	WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);            //packet automation control
	WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);            //packet automation control
	WriteReg(REG_ADDR,VAL_ADDR);                    //device address 
	WriteReg(REG_CHANNR,VAL_CHANNR);                //channel number 
	WriteReg(REG_FSCTRL1,VAL_FSCTRL1);              //frequency synthesizer control 
	WriteReg(REG_FSCTRL0,VAL_FSCTRL0);              //frequency synthesizer control 
	WriteReg(REG_FREQ2,VAL_FREQ2);                  //frequency control word, high byte 
	WriteReg(REG_FREQ1,VAL_FREQ1);                  //frequency control word, middle byte 
	WriteReg(REG_FREQ0,VAL_FREQ0);                  //frequency control word, low byte 
	WriteReg(REG_MDMCFG4,VAL_MDMCFG4);              //modem configuration 
	WriteReg(REG_MDMCFG3,VAL_MDMCFG3);              //modem configuration 
	WriteReg(REG_MDMCFG2,VAL_MDMCFG2);              //modem configuration
	WriteReg(REG_MDMCFG1,VAL_MDMCFG1);              //modem configuration
	WriteReg(REG_MDMCFG0,VAL_MDMCFG0);              //modem configuration 
	WriteReg(REG_DEVIATN,VAL_DEVIATN);              //modem deviation setting 
	WriteReg(REG_MCSM2,VAL_MCSM2);                  //main radio control state machine configuration 
	WriteReg(REG_MCSM1,VAL_MCSM1);                  //main radio control state machine configuration
	WriteReg(REG_MCSM0,VAL_MCSM0);                  //main radio control state machine configuration 
	WriteReg(REG_FOCCFG,VAL_FOCCFG);                //frequency offset compensation configuration
	WriteReg(REG_BSCFG,VAL_BSCFG);                  //bit synchronization configuration
	WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);            //agc control
	WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);            //agc control
	WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);            //agc control
	WriteReg(REG_WOREVT1,VAL_WOREVT1);              //high byte event0 timeout 
	WriteReg(REG_WOREVT0,VAL_WOREVT0);              //low byte event0 timeout 
	WriteReg(REG_WORCTRL,VAL_WORCTRL);              //wake on radio control
	WriteReg(REG_FREND1,VAL_FREND1);                //front end rx configuration 
	WriteReg(REG_FREND0,VAL_FREND0);                //front end tx configuration 
	WriteReg(REG_FSCAL3,VAL_FSCAL3);                //frequency synthesizer calibration 
	WriteReg(REG_FSCAL2,VAL_FSCAL2);                //frequency synthesizer calibration 
	WriteReg(REG_FSCAL1,VAL_FSCAL1);                //frequency synthesizer calibration 
	WriteReg(REG_FSCAL0,VAL_FSCAL0);                //frequency synthesizer calibration 
	WriteReg(REG_RCCTRL1,VAL_RCCTRL1);              //rc oscillator configuration 
	WriteReg(REG_RCCTRL0,VAL_RCCTRL0);              //rc oscillator configuration 
	WriteReg(REG_FSTEST,VAL_FSTEST);                //frequency synthesizer calibration control 
	WriteReg(REG_PTEST,VAL_PTEST);                  //production test 
	WriteReg(REG_AGCTEST,VAL_AGCTEST);              //agc test 
	WriteReg(REG_TEST2,VAL_TEST2);                  //various test settings 
	WriteReg(REG_TEST1,VAL_TEST1);                  //various test settings 
	WriteReg(REG_TEST0,VAL_TEST0);                  //various test settings 
        WriteReg(0x3E,0xFF);
	//WriteReg(REG_PARTNUM,VAL_PARTNUM);              //chip id 
	//WriteReg(REG_VERSION,VAL_VERSION);              //chip id 
	//WriteReg(REG_FREQEST,VAL_FREQEST);              //frequency offset estimate from demodulator 
	//WriteReg(REG_LQI,VAL_LQI);                      //demodulator estimate for link quality 
	//WriteReg(REG_RSSI,VAL_RSSI);                    //received signal strength indication 
	//WriteReg(REG_MARCSTATE,VAL_MARCSTATE);          //main radio control state machine state 
	//WriteReg(REG_WORTIME1,VAL_WORTIME1);            //high byte of wor time 
	//WriteReg(REG_WORTIME0,VAL_WORTIME0);            //low byte of wor time 
	//WriteReg(REG_PKTSTATUS,VAL_PKTSTATUS);          //current gdoxstatus and packet status 
	//WriteReg(REG_VCO_VC_DAC,VAL_VCO_VC_DAC);        //current setting from pll calibration module 
	//WriteReg(REG_TXBYTES,VAL_TXBYTES);              //underflow and number of bytes 
	//WriteReg(REG_RXBYTES,VAL_RXBYTES);              //underflow and number of bytes 
	//WriteReg(REG_RCCTRL1_STATUS,VAL_RCCTRL1_STATUS);//last rc oscillator calibration result 
	//WriteReg(REG_RCCTRL0_STATUS,VAL_RCCTRL0_STATUS);//last rc oscillator calibration result 

}



#endif
