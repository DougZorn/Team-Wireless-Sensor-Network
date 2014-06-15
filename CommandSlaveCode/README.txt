#This is a Readme for Command & Slave node code
# Instructions to get the whole process of setting up the command and slave node code

Command Node:
   When setting up network:
      - set variables for amount of nodes are in the network, including the command node
      - start R program, processing program and reset command node in order to ensure proper data transfer
     
   Duties of Command Node:
      - Obtain packets from all nodes and send all distance data to computer for processing
      - receive all new calculated locations and send out to all nodes for update
      
Slave Node:
   
   When setting up network:
      - set variables for amount of nodes are in the network, including the command node
      - Set the Pin for LED to be V2 or V1 of the code depending on the hardware as they use different pins (on copter or battery board)
      - set the header for read_write.h or V2 of that header depending on the hardware, in both the node_codeV3 and CC2500init.h (on copter or battery board)
      
   Duties of Command Node:
      - send out packets containing sensor data(not used , out of time) and distance from RSSI information
      - eavesdrop on all traffic to get RSSI for distance of all nodes
      - send data to command node
