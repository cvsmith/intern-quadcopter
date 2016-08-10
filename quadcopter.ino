/**This is an example of using the RCLib with a mega and pinchangeint
 * This example isprovided as is by Jantje
 **/

#define NUM_RC_CHANNELS 6 //You need to specify how many pins you want to use
#include "PinChangeInt.h"  //If you need pinchangeint you need to include this header
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={
  A8,A9,A10,A11,A12,A13};//Here I specify I want to listen to pins A8 to A13 of my mega

uint16_t RC_Channel_Value[NUM_RC_CHANNELS]; //This variable will contain the values read from the RC signal

// RC controller variables
float yaw_stick, pitch_stick, roll_stick, collective_stick;

#define YAW_STICK_OFFSET 1600
#define PITCH_STICK_OFFSET 1550
#define ROLL_STICK_OFFSET 1450
#define COLLECTIVE_STICK_OFFSET 1500
#define YAW_STICK_SCALE 935
#define PITCH_STICK_SCALE 896
#define ROLL_STICK_SCALE 935
#define COLLECTIVE_STICK_SCALE 1000

#include "RCLib.h" //This include needs all declarations above. Do not try to move it up or it won't compile

/** plot is a general function I use to output data sets
**/
void plot(int Data1, int Data2, int Data3, int Data4=0, int Data5=0, int Data6=0, int Data7=0, int Data8=0)
{
  Serial.print(Data1); 
  Serial.print(" ");
  Serial.print(Data2); 
  Serial.print(" ");
  Serial.print(Data3); 
  Serial.print(" ");
  Serial.print(Data4); 
  Serial.print(" ");
  Serial.print(Data5); 
  Serial.print(" ");
  Serial.print(Data6); 
  Serial.print(" ");
  Serial.print(Data7); 
  Serial.print(" ");
  Serial.println(Data8); 
}

void setup()
{
  Serial.begin(57600);
  Serial.println(F("Rc serial oscilloscope demo"));
  SetRCInterrupts(); //This method will do all the config for you.
                    //Note some problems will be reported on the serial monitor
  Serial.println(F("Interrupts Set; starting "));
}

void loop()
{
  //Add your repeated code here
  
  int flag;
  if(flag=getChannelsReceiveInfo()) //Here you read the RC flag contains all the channels that have a response
                                    // see duane's excellent articles on how this works
  {
    plot(RC_Channel_Value[0],RC_Channel_Value[1],RC_Channel_Value[2],RC_Channel_Value[3],RC_Channel_Value[4],RC_Channel_Value[5]);
  }
  delay(50); //don't flood your serial monitor

}

