/*
 * quadcopter.ino - Firmware for a basic quadcopter flight controller
 */

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// Yaw
#define K_R 1.5
#define K_PSI 2.5
#define R_MAX 0.5
#define R_DOT_MAX 1.0
#define R_BNDW 2.0
#define NDR_INV_EST 40.0
#define NR_EST 0.2
#define TRIM_TAU 0.1
#define YAW_TRIM_TAU 2.0

#define DELTA_T 0.01
#define E 2.71828

/*
 * DON'T USE THE RIGHT SW
 */

// Motors
Servo motor1, motor2, motor3, motor4;

#define NUM_RC_CHANNELS 5
#include "PinChangeInt.h"
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS] = {A8,A9,A10,A11,A12};

uint16_t RC_Channel_Value[NUM_RC_CHANNELS];

#define TIMEOUT 1000 // ms since last signal before emergency mode
unsigned long last_signal_time; // millis() rolls over after ~50 days

// RC controller variables
#define ROLL_CHAN 0
#define YAW_CHAN 1
#define COLL_CHAN 2
#define PITCH_CHAN 3
#define GROUNDED_SW_CHAN 4

float yaw_stick, pitch_stick, roll_stick, coll_stick;
bool grounded_sw = true;
float omega_1_cmd, omega_2_cmd, omega_3_cmd, omega_4_cmd;
bool cmd_mdl_ic = grounded_sw;

#define YAW_STICK_OFFSET 1600.0
#define PITCH_STICK_OFFSET 1550.0
#define ROLL_STICK_OFFSET 1450.0
#define COLL_STICK_OFFSET 1000.0
#define YAW_STICK_SCALE 446.0
#define PITCH_STICK_SCALE 485.0
#define ROLL_STICK_SCALE 485.0
#define COLL_STICK_SCALE 1000.0
#define GROUNDED_SW_THRESHOLD 1500

#define MOTOR_SCALE 1.1
#define MOTOR_OFFSET 1050

// Control law constants
#define PHI_MAX 0.5
#define PHI_BNDW 1.0
#define P_BNDW 4.0
#define P_MAX 0.5
#define P_DOT_MAX 1.0
#define LP_EST 1.2
#define LDS_INV_EST 2.5
#define ROLL_TRIM_TAU 2.0
#define K_P 3.0
#define K_PHI 6.0
 
#define THETA_MAX 0.5
#define THETA_BNDW 1.0
#define Q_BNDW 4.0
#define Q_MAX 0.5
#define Q_DOT_MAX 1.0
#define MQ_EST 1.2
#define MDB_INV_EST 2.5
#define PITCH_TRIM_TAU 2.0
#define K_Q 3.0
#define K_THETA 6.0


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

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055();
uint8_t system, gyro, accel, mag = 0;   // Calibration status for IMU 0 = no cal, 3 = full cal

void plot(float Data1, float Data2, float Data3, float Data4=0, float Data5=0, float Data6=0, float Data7=0, float Data8=0)
{
  Serial.print(Data1); 
  Serial.print("\t\t");
  Serial.print(Data2); 
  Serial.print("\t\t");
  Serial.print(Data3); 
  Serial.print("\t\t");
  Serial.print(Data4); 
  Serial.print("\t\t");
  Serial.print(Data5); 
  Serial.print("\t\t");
  Serial.print(Data6); 
  Serial.print("\t\t");
  Serial.print(Data7); 
  Serial.print("\t\t");
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
  coll_stick = (RC_Channel_Value[COLL_CHAN] - COLL_STICK_OFFSET) / COLL_STICK_SCALE;
  
  grounded_sw = RC_Channel_Value[GROUNDED_SW_CHAN] > GROUNDED_SW_THRESHOLD ? true : false;
  cmd_mdl_ic = grounded_sw;
}

void send_motor_cmds()
{
  motor1.writeMicroseconds((omega_1_cmd + MOTOR_OFFSET) * MOTOR_SCALE);
  motor2.writeMicroseconds((omega_2_cmd + MOTOR_OFFSET) * MOTOR_SCALE);
  motor3.writeMicroseconds((omega_3_cmd + MOTOR_OFFSET) * MOTOR_SCALE);
  motor4.writeMicroseconds((omega_4_cmd + MOTOR_OFFSET) * MOTOR_SCALE);
}
  
float integral(float x, float y_prev, bool ic_logic, float ic_var, float dt)
{
  float y;
  if (ic_logic == 1) {
    y = ic_var;
  } else {
    y = y_prev + x * dt;
  }
  return y;
}

float lag(float x, float y_prev, float tau, bool ic_logic, float ic_var, float dt)
{
  float y;
  float K;
  if (ic_logic == 1) {
    y = ic_var;
  } else {
    K = 1 - pow(E, -1 * dt / tau);
    y = K * x + (1 - K) * y_prev;
  }
}

float slimit(float x, float limit)
{
  if (x >= limit) {
    return limit;
  } else if (x <= -1 * limit) {
    return -1 * limit;
  } else {
    return x;
  }
}


void MarkCode2(float roll_stick, float pitch_stick, float yaw_stick, float
               coll_stick, float p, float q, float r, float phi, float theta,
               float psi, bool IC_RST, float *omega_1_cmd, float
               *omega_2_cmd, float *omega_3_cmd, float *omega_4_cmd)
{
  float lon_diff_omega, lat_diff_omega, dir_diff_omega;
  float collective_omega;
  float r_cmd_raw, r_dot_cmd_raw, r_dot_filt, r_dot_cmd, yaw_feedfwd;
  float yaw_trim_add, yaw_trim_raw, yaw_sum;
  float r_err, psi_err, yaw_feedback;
  float phi_cmd_raw, p_cmd_raw, p_cmd_limited, p_dot_cmd_raw, p_dot_cmd;
  float p_dot_filt, roll_feedfwd, roll_trim_add, roll_trim_raw, p_err, roll_feedback, phi_err;
  float theta_cmd_raw, q_cmd_raw, q_cmd_limited, q_dot_cmd_raw, q_dot_cmd;
  float q_dot_filt, pitch_feedfwd, pitch_trim_add, pitch_trim_raw, q_err, pitch_feedback, theta_err;

  // Yaw
  r_cmd_raw = yaw_stick * R_MAX;
  r_dot_cmd_raw = R_BNDW * (r_cmd_raw - r_cmd);
  psi_cmd = integral(r_cmd, psi_cmd, cmd_mdl_ic, psi, DELTA_T);
  yaw_feedfwd = r_dot_cmd + r_cmd * NR_EST;
  
  r_filt = lag(r, r_filt, TRIM_TAU, cmd_mdl_ic, r, DELTA_T);
  r_dot_filt = (r - r_filt) / TRIM_TAU;
  yaw_trim_add = r_filt * NR_EST + r_dot_filt;
  yaw_sum_filt = lag(yaw_sum, yaw_sum_filt, TRIM_TAU, cmd_mdl_ic, r, DELTA_T); 
  yaw_trim_raw = yaw_sum_filt - yaw_trim_add;
  yaw_trim = lag(yaw_trim_raw, yaw_trim, YAW_TRIM_TAU, cmd_mdl_ic, 0, DELTA_T);

  r_err = r_cmd - r;
  psi_err = psi_cmd - psi;
  yaw_feedback = r_err * K_R + psi_err * K_PSI;
  
  yaw_sum = yaw_feedfwd + yaw_feedback + yaw_trim * 0;
  dir_diff_omega = yaw_sum * NDR_INV_EST;
  
  // Roll
  phi_cmd_raw = roll_stick * PHI_MAX;
  p_cmd_raw = PHI_BNDW * (phi_cmd_raw - phi_cmd);
  p_cmd_limited = slimit(p_cmd_raw, P_MAX);
  p_dot_cmd_raw = P_BNDW * (p_cmd_limited - p_cmd);
  p_dot_cmd = slimit(p_dot_cmd_raw, P_DOT_MAX);
  p_cmd = integral(p_dot_cmd, p_cmd, cmd_mdl_ic, 0, DELTA_T);
  phi_cmd = integral(p_cmd, phi_cmd, cmd_mdl_ic, phi, DELTA_T);
  roll_feedfwd = p_dot_cmd + p_cmd * LP_EST;
  
  p_filt = lag(p, p_filt, TRIM_TAU, cmd_mdl_ic, p, DELTA_T);
  p_dot_filt = (p - p_filt) / TRIM_TAU;
  roll_trim_add = p_filt * LP_EST + p_dot_filt;
  roll_sum_filt = lag(roll_sum, roll_sum_filt, TRIM_TAU, cmd_mdl_ic, p, DELTA_T); 
  roll_trim_raw = roll_sum_filt - roll_trim_add;
  roll_trim = lag(roll_trim_raw, roll_trim, ROLL_TRIM_TAU, cmd_mdl_ic, 0, DELTA_T);

  p_err = p_cmd - p;
  psi_err = psi_cmd - psi;
  roll_feedback = p_err * K_P + phi_err * K_PHI;
  
  roll_sum = roll_feedfwd + roll_feedback + roll_trim * 0;
  lat_diff_omega = roll_sum * LDS_INV_EST;
  
  // Pitch
  theta_cmd_raw = pitch_stick * THETA_MAX;
  q_cmd_raw = THETA_BNDW * (theta_cmd_raw - theta_cmd);
  q_cmd_limited = slimit(q_cmd_raw, Q_MAX);
  q_dot_cmd_raw = Q_BNDW * (q_cmd_limited - q_cmd);
  q_dot_cmd = slimit(q_dot_cmd_raw, Q_DOT_MAX);
  q_cmd = integral(q_dot_cmd, q_cmd, cmd_mdl_ic, 0, DELTA_T);
  theta_cmd = integral(q_cmd, theta_cmd, cmd_mdl_ic, theta, DELTA_T);
  pitch_feedfwd = q_dot_cmd + q_cmd * MQ_EST;
  
  q_filt = lag(p, q_filt, TRIM_TAU, cmd_mdl_ic, p, DELTA_T);
  q_dot_filt = (q - q_filt) / TRIM_TAU;
  pitch_trim_add = q_filt * MQ_EST + q_dot_filt;
  pitch_sum_filt = lag(pitch_sum, pitch_sum_filt, TRIM_TAU, cmd_mdl_ic, p, DELTA_T); 
  pitch_trim_raw = pitch_sum_filt - pitch_trim_add;
  pitch_trim = lag(pitch_trim_raw, pitch_trim, PITCH_TRIM_TAU, cmd_mdl_ic, 0, DELTA_T);

  q_err = q_cmd - p;
  psi_err = psi_cmd - psi;
  pitch_feedback = q_err * K_Q + theta_err * K_THETA;
  
  pitch_sum = pitch_feedfwd + pitch_feedback + pitch_trim * 0;
  lon_diff_omega = pitch_sum * MDB_INV_EST;
  
  // Collective
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

void get_imu_vals()
{
  // read the IMU data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);      // produces variables euler.x, euler.y, euler.z
  imu::Vector<3> ar = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // produces variables la.x, la.y, la.z
  bno.getCalibration(&system, &gyro, &accel, &mag);                         // produces imu sesnsor calibration variables
  
  p = ar.x();
  q = -1 * ar.y();
  r = -1 * ar.z();
  
  phi = -1 * euler.z() * 0.01745;
  theta = euler.y() * 0.01745;
  psi = euler.x() * 0.01745;
  
}

void clear_rc_vals()
{
  roll_stick, pitch_stick, yaw_stick, coll_stick = 0;
}

void setup()
{
  // Turn on the IMU:
  bno.begin();
  
  // Motors
  motor1.attach(2, 1000, 2000);
  motor2.attach(3, 1000, 2000);
  motor3.attach(4, 1000, 2000);
  motor4.attach(5, 1000, 2000);
  motor1.writeMicroseconds(1001);
  motor2.writeMicroseconds(1001);
  motor3.writeMicroseconds(1001);
  motor4.writeMicroseconds(1001);
  
  Serial.begin(57600);
  Serial.println(F("Rc serial oscilloscope demo"));
  SetRCInterrupts(); //This method will do all the config for you.
                    //Note some problems will be reported on the serial monitor
  Serial.println(F("Interrupts Set; starting "));
}

/*
 * loop() - Check RC and IMU, calculate and send motor commands
 *
 */
void loop()
{
  get_rc_vals();  
  get_imu_vals();
  MarkCode2(roll_stick, pitch_stick, yaw_stick, coll_stick, 
            p, q, r, phi, theta, psi, grounded_sw, 
            &omega_1_cmd, &omega_2_cmd, &omega_3_cmd, &omega_4_cmd);
  
  send_motor_cmds();
  // plot(p, q, r, phi, theta, psi);
  // plot(system, gyro, accel, mag);
  int flag;
  if (flag = getChannelsReceiveInfo())
  {
    last_signal_time = millis();
    plot(roll_stick, pitch_stick, yaw_stick, coll_stick, omega_1_cmd, omega_2_cmd, omega_3_cmd, omega_4_cmd);
  }
  
  // If the receiver has been disconnected, reset the RC values to 0
  if (last_signal_time - millis() > TIMEOUT)
  {
    clear_rc_vals();
  }
  
  delay(10);
}

