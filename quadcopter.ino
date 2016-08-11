/**This is an example of using the RCLib with a mega and pinchangeint
 * This example isprovided as is by Jantje
 * DON'T USE THE RIGHT SW
 */

#define NUM_RC_CHANNELS 5 //You need to specify how many pins you want to use
#include "PinChangeInt.h"  //If you need pinchangeint you need to include this header
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={A8,A9,A10,A11,A12};

uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

// RC controller variables
#define ROLL_CHAN 0
#define YAW_CHAN 1
#define coll_CHAN 2
#define PITCH_CHAN 3
#define GROUNDED_SW_CHAN 4

float yaw_stick, pitch_stick, roll_stick, coll_stick;
bool grounded_sw;

#define YAW_STICK_OFFSET 1600.0
#define PITCH_STICK_OFFSET 1550.0
#define ROLL_STICK_OFFSET 1450.0
#define coll_STICK_OFFSET 1000.0
#define YAW_STICK_SCALE 446.0
#define PITCH_STICK_SCALE 485.0
#define ROLL_STICK_SCALE 485.0
#define coll_STICK_SCALE 1000.0
#define GROUNDED_SW_THRESHOLD 1500

// IMU variables
float p, q, r, phi, theta, psi;

#include "RCLib.h" //This include needs all declarations above. Do not try to move it up or it won't compile

float q_cmd;
float theta_cmd;
float q_filt;
float pitch_sum_filt;
float pitch_sum;
float pitch_trim;
float trim_lon_diff_omega;
float p_cmd;
float phi_cmd;
float p_filt;
float roll_sum_filt;
float roll_sum;
float roll_trim;
float trim_lat_diff_omega;
float r_cmd;
float psi_cmd;
float r_filt;
float yaw_sum_filt;
float yaw_sum;
float yaw_trim;
float trim_dir_diff_omega;
float coll_sensitivity = 800;


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
 * Populate rc floats and bools with scaled, offset RC signals
 */
void get_rc_vals()
{
  yaw_stick = (RC_Channel_Value[YAW_CHAN] - YAW_STICK_OFFSET) / YAW_STICK_SCALE;
  pitch_stick = (RC_Channel_Value[PITCH_CHAN] - PITCH_STICK_OFFSET) / PITCH_STICK_SCALE;
  roll_stick = (RC_Channel_Value[ROLL_CHAN] - ROLL_STICK_OFFSET) / ROLL_STICK_SCALE;
  coll_stick = (RC_Channel_Value[coll_CHAN] - coll_STICK_OFFSET) / coll_STICK_SCALE;
  grounded_sw = RC_Channel_Value[GROUNDED_SW_CHAN] > GROUNDED_SW_THRESHOLD ? true : false;
}

void MarkCode2(float roll_stick, float pitch_stick, float yaw_stick, float
               coll_stick, float p, float q, float r, float phi, float theta,
               float psi, bool IC_RST, float *omega_1_cmd, float
               *omega_2_cmd, float *omega_3_cmd, float *omega_4_cmd)
{
  float lon_diff_omega;
  float lat_diff_omega;
  float dir_diff_omega;
  float collective_omega;

  collective_omega = coll_stick * coll_sensitivity;
  *omega_1_cmd = ((collective_omega + lat_diff_omega) + lon_diff_omega) +
    dir_diff_omega;
  *omega_2_cmd = ((collective_omega - lat_diff_omega) + lon_diff_omega) -
    dir_diff_omega;
  *omega_3_cmd = ((collective_omega - lat_diff_omega) - lon_diff_omega) +
    dir_diff_omega;
  *omega_4_cmd = ((collective_omega + lat_diff_omega) - lon_diff_omega) -
    dir_diff_omega;
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
  get_rc_vals();  
  //get_imu_vals();
  float omega_1_cmd, omega_2_cmd, omega_3_cmd, omega_4_cmd;
  MarkCode2(roll_stick, pitch_stick, yaw_stick, coll_stick, 
            p, q, r, phi, theta, psi, grounded_sw, 
            &omega_1_cmd, &omega_2_cmd, &omega_3_cmd, &omega_4_cmd);
            
  //send_motor_cmds();
  
  int flag;
  if(flag=getChannelsReceiveInfo()) // see duane's excellent articles on how this works
  {
    plot(omega_1_cmd, omega_2_cmd, omega_3_cmd, omega_4_cmd);
  }
  //delay(50);

}

