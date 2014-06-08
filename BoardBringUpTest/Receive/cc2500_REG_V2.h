 /*Sync word qualifier mode = No preamble/sync 
 CRC autoflush = false 
 Channel spacing = 199.951172 
 Data format = Normal mode 
 Data rate = 2.39897 
 RX filter BW = 203.125000 
 Preamble count = 4 
 Whitening = true 
 Address config = Address check and 0 (0x00) broadcast 
 Carrier frequency = 2433.599762 
 Device address = 255 
 TX power = 1 
 Manchester enable = false 
 CRC enable = true 
 Deviation = 38.085938 
 Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
 Packet length = 255 
 Modulation format = 2-FSK 
 Base frequency = 2432.999908 
 Modulated = true 
 Channel number = 3 
 PA table
*/

#ifndef CC2500_REG_V2_H
#define CC2500_REG_V2_H
 
#define PA_TABLE {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}
#define REG_IOCFG2                0x0000    //GDO2Output Pin Configuration 
#define REG_IOCFG1                0x0001    //GDO1Output Pin Configuration 
#define REG_IOCFG0                0x0002    //GDO0Output Pin Configuration 
#define REG_FIFOTHR               0x0003    //RX FIFO and TX FIFO Thresholds
#define REG_SYNC1                 0x0004    //Sync Word, High Byte 
#define REG_SYNC0                 0x0005    //Sync Word, Low Byte 
#define REG_PKTLEN                0x0006    //Packet Length: This is max legnth when PKTCTRL0 = 1 
#define REG_PKTCTRL1              0x0007    //Packet Automation Control
#define REG_PKTCTRL0              0x0008    //Packet Automation Control
#define REG_ADDR                  0x0009    //Device Address 
#define REG_CHANNR                0x000A    //Channel Number 
#define REG_FSCTRL1               0x000B    //Frequency Synthesizer Control 
#define REG_FSCTRL0               0x000C    //Frequency Synthesizer Control 
#define REG_FREQ2                 0x000D    //Frequency Control Word, High Byte 
#define REG_FREQ1                 0x000E    //Frequency Control Word, Middle Byte 
#define REG_FREQ0                 0x000F    //Frequency Control Word, Low Byte 
#define REG_MDMCFG4               0x0010    //Modem Configuration 
#define REG_MDMCFG3               0x0011    //Modem Configuration 
#define REG_MDMCFG2               0x0012    //Modem Configuration
#define REG_MDMCFG1               0x0013    //Modem Configuration
#define REG_MDMCFG0               0x0014    //Modem Configuration 
#define REG_DEVIATN               0x0015    //Modem Deviation Setting 
#define REG_MCSM2                 0x0016    //Main Radio Control State Machine Configuration 
#define REG_MCSM1                 0x0017    //Main Radio Control State Machine Configuration
#define REG_MCSM0                 0x0018    //Main Radio Control State Machine Configuration 
#define REG_FOCCFG                0x0019    //Frequency Offset Compensation Configuration
#define REG_BSCFG                 0x001A    //Bit Synchronization Configuration
#define REG_AGCCTRL2              0x001B    //AGC Control
#define REG_AGCCTRL1              0x001C    //AGC Control
#define REG_AGCCTRL0              0x001D    //AGC Control
#define REG_WOREVT1               0x001E    //High Byte Event0 Timeout 
#define REG_WOREVT0               0x001F    //Low Byte Event0 Timeout 
#define REG_WORCTRL               0x0020    //Wake On Radio Control
#define REG_FREND1                0x0021    //Front End RX Configuration 
#define REG_FREND0                0x0022    //Front End TX configuration 
#define REG_FSCAL3                0x0023    //Frequency Synthesizer Calibration 
#define REG_FSCAL2                0x0024    //Frequency Synthesizer Calibration 
#define REG_FSCAL1                0x0025    //Frequency Synthesizer Calibration 
#define REG_FSCAL0                0x0026    //Frequency Synthesizer Calibration 
#define REG_RCCTRL1               0x0027    //RC Oscillator Configuration 
#define REG_RCCTRL0               0x0028    //RC Oscillator Configuration 
#define REG_FSTEST                0x0029    //Frequency Synthesizer Calibration Control 
#define REG_PTEST                 0x002A    //Production Test 
#define REG_AGCTEST               0x002B    //AGC Test 
#define REG_TEST2                 0x002C    //Various Test Settings 
#define REG_TEST1                 0x002D    //Various Test Settings 
#define REG_TEST0                 0x002E    //Various Test Settings 
#define REG_PARTNUM               0x0030    //Chip ID 
#define REG_VERSION               0x0031    //Chip ID 
#define REG_FREQEST               0x0032    //Frequency Offset Estimate from Demodulator 
#define REG_LQI                   0x0033    //Demodulator Estimate for Link Quality 
#define REG_RSSI                  0x0034    //Received Signal Strength Indication 
#define REG_MARCSTATE             0x0035    //Main Radio Control State Machine State 
#define REG_WORTIME1              0x0036    //High Byte of WOR Time 
#define REG_WORTIME0              0x0037    //Low Byte of WOR Time 
#define REG_PKTSTATUS             0x0038    //Current GDOxStatus and Packet Status 
#define REG_VCO_VC_DAC            0x0039    //Current Setting from PLL Calibration Module 
#define REG_TXBYTES               0x003A    //Underflow and Number of Bytes 
#define REG_RXBYTES               0x003B    //Underflow and Number of Bytes 
#define REG_RCCTRL1_STATUS        0x003C    //Last RC Oscillator Calibration Result 
#define REG_RCCTRL0_STATUS        0x003D    //Last RC Oscillator Calibration Result 

#endif
