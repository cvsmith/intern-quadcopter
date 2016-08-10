/**This is an example of using the RCLib with a mega and pinchangeint
 * This example isprovided as is by Jantje
 **/

#define NUM_RC_CHANNELS 6 //You need to specify how many pins you want to use
#include "PinChangeInt.h"  //If you need pinchangeint you need to include this header
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={
  A8,A9,A10,A11,A12,A13};//Here I specify I want to listen to pins A8 to A13 of my mega

uint16_t RC_Channel_Value[NUM_RC_CHANNELS]; //This variable will contain the values read from the RC signal

// RC controller variables
#define ROLL_CHAN 0
#define YAW_CHAN 1
#define PITCH_CHAN 3
#define COLLECTIVE_CHAN 2

float yaw_stick, pitch_stick, roll_stick, collective_stick;

#define YAW_STICK_OFFSET 1600.0
#define PITCH_STICK_OFFSET 1550.0
#define ROLL_STICK_OFFSET 1450.0
#define COLLECTIVE_STICK_OFFSET 1000.0 // Collective goes from 0 - 1
#define YAW_STICK_SCALE 446.0
#define PITCH_STICK_SCALE 485.0
#define ROLL_STICK_SCALE 485.0
#define COLLECTIVE_STICK_SCALE 1000.0

#include "RCLib.h" //This include needs all declarations above. Do not try to move it up or it won't compile

/** plot is a general function I use to output data sets
  **/
void plot(float Data1, float Data2, float Data3, float Data4=0, float Data5=0, float Data6=0, float Data7=0, float Data8=0)
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

/*
 * Populate stick floats with scaled, offset RC signals
 */
void get_stick_vals()
{
  yaw_stick = (RC_Channel_Value[YAW_CHAN] - YAW_STICK_OFFSET) / YAW_STICK_SCALE;
  pitch_stick = (RC_Channel_Value[PITCH_CHAN] - PITCH_STICK_OFFSET) / PITCH_STICK_SCALE;
  roll_stick = (RC_Channel_Value[ROLL_CHAN] - ROLL_STICK_OFFSET) / ROLL_STICK_SCALE;
  collective_stick = (RC_Channel_Value[COLLECTIVE_CHAN] - COLLECTIVE_STICK_OFFSET) / COLLECTIVE_STICK_SCALE;
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
  get_stick_vals();
  
  int flag;
  if(flag=getChannelsReceiveInfo()) //Here you read the RC flag contains all the channels that have a response
                                    // see duane's excellent articles on how this works
  {
    plot(yaw_stick, pitch_stick, roll_stick, collective_stick);
    //plot(RC_Channel_Value[YAW_CHAN], RC_Channel_Value[PITCH_CHAN], RC_Channel_Value[ROLL_CHAN]);
  }
  delay(50); //don't flood your serial monitor

}

