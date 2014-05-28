#include <string.h>

char userCommand[500];
char *arg1;
char *arg2;
char *i;


void setup()
{
  Serial.begin(9600);
  
}

void loop () {
  
  if(Serial.available()){
    arg1 =NULL;
    arg2 = NULL;
    for(int x =0; x<500;x++){
      userCommand[x]=NULL;
    }
    int x =0;
    delay(20);
    do{
      userCommand[x++] = Serial.read();
    }while(Serial.available());
    
    arg1 = strtok(userCommand, " :\n\r");
    Serial.write(arg1);
    Serial.println("");
    arg2 = strtok(NULL, " :\n\r");
    Serial.write(arg2);
    Serial.println("");
    
    if(!strcmp(arg1, "shutdown")){ 
      Serial.println("You typed shutdown");
    }else if(!strcmp(arg1, "land")){
      Serial.println("You typed land");
    }else if(!strcmp(arg1, "flight")){
      Serial.println("You typed flight");
    }else{
      Serial.println("You typed in shit");
    }
  }
  
}
