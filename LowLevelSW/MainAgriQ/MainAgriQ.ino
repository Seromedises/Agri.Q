#include <SD.h>
#include <SPI.h>
#include <Encoder.h>
#include<Wire.h>
#include<ros.h>
#include<sensor_msgs/Imu>

#define DEBUGBAUD 115200
#define ovf 65535

// LOOP TIME VARIABLES
long time_old = 0;
long time_oldIMU = 0;
long time_oldLOG = 0;
float time_sample = 0.0;

//joystick variables
struct JoyRx {
  const byte PIN_J1updwn;
  volatile uint16_t VALUE_J1updwn;
  volatile uint32_t timer_J1updwn = 0;
  
  const byte PIN_J1leftright;
  volatile uint16_t VALUE_J1leftright;
  volatile uint32_t timer_J1leftright = 0;

  const byte PIN_J2updwn;
  volatile uint16_t VALUE_J2updwn;
  volatile uint32_t timer_J2updwn = 0;
  
  
  const byte PIN_J2leftright;
  volatile uint16_t VALUE_J2leftright;
  volatile uint32_t timer_J2leftright = 0;
  
  const byte PIN_SW1;
  volatile uint32_t VALUE_SW1;
  volatile uint32_t timer_SW1 = 0;
  

  const byte PIN_SW2;
  volatile uint16_t VALUE_SW2;
  volatile uint32_t timer_SW2 = 0;
  };
#define RXPITCHPIN 28
#define RXROLLPIN 29
#define RXLONGPIN 30
#define RXYAWPIN 27
#define RXYSW1PIN 26
#define RXYSW2PIN 25
struct JoyRx AgriQJoyRx = {RXPITCHPIN, 1500, 0, RXROLLPIN, 1500, 0, RXLONGPIN, 1500, 0, RXYAWPIN, 1500, 0, RXYSW1PIN, 0, 0, RXYSW2PIN, 0, 0};

const uint16_t J2ud_max = 1902; // measured value
const uint16_t J2ud_zero = 1504; // measured value
const uint16_t J2ud_min = 1105; // measured value

const uint16_t J2lr_max = 1895; // measured value
const uint16_t J2lr_zero = 1497; // measured value
const uint16_t J2lr_min = 1099; // measured value

float lambda = 0.0; // Longitudinal speed mapping input variable [-1,+1]
const float lambda_dead = 0.05; // lambda half deadband amplitude [0,lambda_sat[
const float lambda_sat = 1.0; // lambda saturation threshold [lambda_dead,1]
float nu = 0.0; // yaw rate mapping input variable [-1,+1]
const float nu_dead = 0.15; // lambda half deadband amplitude [0,lambda_sat[
const float nu_sat = 1.0; // lambda saturation threshold [lambda_dead,1]
float vlong_lambda = 0.0; // Longitudinal speed mapping output variable (cubic smoothing)
const float vlong_max = 1.2852; // m/s
float yawrate_nu = 0.0; // yaw rate mapping output variable (cubic smoothing)
const float yawrate_max = 1.08; //rad/s   // real max 3.12 rad/s

// GEOMETRIC PARAMETERS
const float r_wheel = 0.2032; //m wheel radius (diameter 16" tyre = 406.4 mm)
const float track = 0.845; //m lateral axle length (right-left wheels distance) 

// MOTORS ENABLE & Brake VARIABLES
#define EnableFrontMotorsPin 16
#define EnableRearMotorsPin 24
#define BrakeFrontMotorsPin 19
#define BrakeRearMotorsPin 18
bool StateAuto = 0; // Flag to check if automatic mode is inserted (1) or not (0)
bool EnableFrontMotors = 0;
bool BrakeFrontMotors = 0;
bool EnableRearMotors = 0;
bool BrakeRearMotors = 0;

// PITCH MOTOR VARIABLES
#define DirPitchPin 14
#define BrakePitchPin 17
#define EnPitchPin 15

// ROLL MOTOR VARIABLES
#define EnRollPin 8
#define PWMRollPin 9
#define DirRollPin 10

// TRACTION MOTORS VARIABLES
const float omega_max = 314.0; // rad/s
const float torque_max = 0.55 *1000 *2.0 ; // Nmm
struct TractionMotor {
  const byte EncA_PIN;
  const byte EncB_PIN;

  const byte PWM_PIN;
  const byte Current_PIN;
  
  float reference = 0.0;
  float omega_measured = 0.0;
  float omega_measuredOLD = 0.0;
  float current_measured = 0.0;

  long EncCountOLD = 0;
  volatile long EncCount = 0;

  const int CPR; // encoder resolution
  const float tau; //transmission ratio motor to wheel

  float odometry = 0.0;

};
struct PID_var {
  float error;      //-   error between refrence and measured value
  float ctrl;     //-   control output
  float INT;        //      Integral part
  float error_old;  //-   prevoius error
  float DER;        //      derivative part
  float GainP;      // Proportional gain
  float GainI;      // Integral gain
  float GainD;      // Derivative gain
};

// Front Left Traction Motor
#define EncAMFLPin 0
#define EncBMFLPin 1
#define PwmMFLPin 7
#define CurrBMFLPin A21
struct TractionMotor MFL = {EncAMFLPin, EncBMFLPin, PwmMFLPin, CurrBMFLPin, 0.0 ,0.0, 0.0, 0.0, 0, 0, 400, 15.88*3.0, 0.0};
struct PID_var PID_MFL = {0.0, 0.0, 0.0, 0.0, 0.0, 6, 0.01, 0.0};

Encoder Enc_MFL(MFL.EncA_PIN,MFL.EncB_PIN);

// Front Right Traction Motor
#define EncAMFRPin 2
#define EncBMFRPin 3
#define PwmMFRPin 6
#define CurrBMFRPin A22
struct TractionMotor MFR = {EncAMFRPin, EncBMFRPin, PwmMFRPin, CurrBMFRPin, 0.0 ,0.0, 0.0, 0.0, 0, 0, 400, 15.88*3.0, 0.0};
struct PID_var PID_MFR = {0.0, 0.0, 0.0, 0.0, 0.0, 6, 0.01, 0.0};

Encoder Enc_MFR(MFR.EncA_PIN,MFR.EncB_PIN);

// Rear Left Traction Motor
#define EncAMRLPin 23
#define EncBMRLPin 22
#define PwmMRLPin 4
#define CurrBMRLPin A13
struct TractionMotor MRL = {EncAMRLPin, EncBMRLPin, PwmMRLPin, CurrBMRLPin, 0.0 ,0.0, 0.0, 0.0, 0, 0, 400, 28.93, 0.0};

Encoder Enc_MRL(MRL.EncA_PIN,MRL.EncB_PIN);

// Rear Right Traction Motor
#define EncAMRRPin 21
#define EncBMRRPin 20
#define PwmMRRPin 4
#define CurrBMRRPin A12
struct TractionMotor MRR = {EncAMRRPin, EncBMRRPin, PwmMRRPin, CurrBMRRPin, 0.0 ,0.0, 0.0, 0.0, 0, 0, 400, 28.93, 0.0};

Encoder Enc_MRR(MRR.EncA_PIN,MRR.EncB_PIN);

float Current_F_filt = 0.0; // use to control the rear motors

const float distribution_ratio = 0.62;

/* LOG ON SD IS CURRENTLY DISABLED. TO LOG DATA, USE A SERIAL LOGGER ON THE REMOTELY CONTROLLED PC
//   SD
File myLOG;
String dataString;                                                                        //variabile stringa per il log
const char* logFile = "AgriQdatalog.txt";                                                      //constant char* type to be passed to the SD methods
String logName = String(logFile); 
*/
bool enb_DATALOG = 0; //variabile booleana per abilitare/disabilitare (1/0) il LOG


// BATTERY & PANEL
#define PanelCurrentPin A17
#define PanelVoltagePin A18
#define BatteryCurrentPin A19
#define BatteryVoltagePin A20

struct power_var {
  float PanelCurrent = 0.0;
  const float m_PanelCurrent = 10.3;//[A/V]
  const float q_PanelCurrent = -25.75;//[A]
  
  float PanelVoltage = 0.0;
  const float m_PanelVoltage = 0.03;//[V/bit]
  const float q_PanelVoltage = 7.03;//[V]

  float BatteryCurrent = 0.0;
  const float m_BatteryCurrent = 31.457;//[A/V]
  const float q_BatteryCurrent = -78.625; //[A]
  
  float BatteryVoltage = 0.0;
  const float m_BatteryVoltage = 0.03;//[V/bit]
  const float q_BatteryVoltage = 6.83;//[V]
};

struct power_var AgriQPower = {0.0, 10.3, -25.75, 0.03, 7.03, 31.457, -78.625, 0.03, 6.83};

// POTENTIOMETERS
#define Roll_measuredPin A26
#define RollBack_measuredPin A10
#define Pitch_measuredPin A25
#define Yaw_measuredPin A11

float Roll_measured = 0.0; //rad
float RollBack_measured = 0.0; //rad
float Pitch_measured = 0.0; //rad
float Yaw_measured = 0.0; //rad

float Roll_reference = 0.0; //rad
float Pitch_reference = 0.0; //rad

// RECEIVED DATA
const byte numChars = 32;        // if the distance is in metre and between +-99.99, and the two angles are in radians and within +-0
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

boolean newData = false;
bool PanelAutoRoll = 0;   //if 1, roll angle is controlled by a closed loop system to reach the desired angle
bool PanelAutoPitch = 0;  //if 1, pitch angle is controlled by a closed loop system to reach the desired angle
bool AgriQAutoAdvance = 0;//if 1, advance distance is controlled by a closed loop system to reach the desired angle

float Advance_reference = 0.0; //m

// IMU

struct 6dofIMU {
  const int MPU_addr=0x68;  // I2C address of the MPU-6050
  int16_t AcX = 0,AcY = 0,AcZ = 0,Tmp = 0,GyX = 0,GyY = 0,GyZ = 0; // RAW DATA FROM REGISTERS
  float aX = 0.0, aY = 0.0, aZ = 0.0,TmpC = 0.0, gX = 0.0, gY = 0.0, gZ = 0.0; //FLOAT DATA
};

struct 6dofIMU AgriQFIMU = {0x68, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// ROS

ros::NodeHandle  nh;

// ROS publisher:
sensor_msgs::Imu IMU_msg;
ros::Publisher AgriQFIMUtopic("AgriQFIMUtopic", &IMU_msg);

/////////////
//  SETUP  //
/////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(DEBUGBAUD);
  analogWriteResolution(12);

  Rx_init();
  EnBrake_init();
  Roll_init();
  Pitch_init();
  TractionMotors_init();
  IMU_init();
  //SDinit();

  nh.initNode();
  nh.advertise(AgriQFIMUtopic);

}

/////////////////
//  MAIN LOOP  //
/////////////////
void loop() {
  
  if (millis() - time_old >= 10){
      time_sample = (millis()-time_old)/1000.0;
      time_old = millis();
      
      // Navigation joystick mapping
      lambda = mapfloat(AgriQJoyRx.VALUE_J2updwn,J2ud_min,J2ud_max,-1.0,+1.0);
      nu = mapfloat(AgriQJoyRx.VALUE_J2leftright,J2lr_min,J2lr_max,-1.0,+1.0);

      vlong_lambda = cubicmapping(lambda, lambda_dead, lambda_sat, vlong_max);
      yawrate_nu = cubicmapping(nu, nu_dead, nu_sat, -yawrate_max); // NOTE: -yawrate_max to get positive yaw rate turning left (anti clock wise)
     
      // Set Motor Brake and Enable PINS
      
      StateAuto = EnBrake_outcmd(AgriQJoyRx.VALUE_SW1, AgriQJoyRx.VALUE_SW2);
      
      //enb_DATALOG = StateAuto; // ACTIVABLE LOG REMOVED

      GetBaseMeasures(); // Get angles, motor angular speeds, motor currents, battery voltage, current comsumption, PV panels current


      IMU_msg.orientation = {0, 0, 0, 0};
      IMU_msg.orientation_covariance = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
      IMU_msg.angular_velocity = {AgriQFIMU.gX, AgriQFIMU.gY, AgriQFIMU.gY};
      IMU_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      IMU_msg.linear_acceleration = {AgriQFIMU.aX, AgriQFIMU.aY, AgriQFIMU.aZ};
      IMU_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

      AgriQFIMUtopic.publish( &IMU_msg );
      nh.spinOnce();



      ////////////////// ADVANCE, PITCH AND ROLL CLOSED LOOP MODE ///////////////////
      // ADAPT THIS SECTION TO ROS MSGs

      if (StateAuto == 1){
        
        
        
        // Panel Roll
        PanelAutoRoll = Roll_ClosedLoopOutcmd(Roll_reference, Roll_measured, PanelAutoRoll); // Closed loop control based on PanelAutoRoll
      
        // Panel Pitch
        PanelAutoPitch = Pitch_ClosedLoopOutcmd(Pitch_reference, Pitch_measured, PanelAutoPitch); // Closed loop control based on PanelAutoPitch
            
        ////// Measure Motors data (speeds, currents)
        ////// Front Motors Controller

        //ODOMETRY CONTROLLER -> lambda = f(advance); nu = 0 (straight);
        lambda = AgriQAutoAdvanceFwd(Advance_reference, 0.5*(MFL.odometry + MFR.odometry), AgriQAutoAdvance);
        vlong_lambda = cubicmapping(lambda, lambda_dead, lambda_sat, vlong_max);
        nu = 0;
        
        MFront_ReferenceOmega(lambda, nu, vlong_lambda, yawrate_nu);

        PID_MFL = PID_Ctrl(time_sample*1000, MFL.reference, MFL.omega_measured,MFL.omega_measuredOLD , PID_MFL, -torque_max, torque_max); // ts in ms
        PID_MFL.ctrl = mapfloat(PID_MFL.ctrl, -torque_max, torque_max, 4095, 0); //left motor: 0 fwd  4095 bwd  no current 2000
        MFL_outcmd(PID_MFL.ctrl -100);
      
        PID_MFR = PID_Ctrl(time_sample, MFR.reference, MFR.omega_measured,MFR.omega_measuredOLD , PID_MFR, -torque_max, torque_max); // ts in s
        PID_MFR.ctrl = mapfloat(PID_MFR.ctrl, -torque_max, torque_max, 4095, 0); //left motor: 0 fwd  4095 bwd  no current 2000
        MFR_outcmd(PID_MFR.ctrl -120);

        ////// Rear Motor Controller    
        if (AgriQJoyRx.VALUE_SW2 > 1600 && lambda > 0.1){
        
          Current_F_filt = 0.5*(PID_MFL.ctrl + PID_MFR.ctrl);
      
          MRL.reference = (1/distribution_ratio -1)*(Current_F_filt*MFL.tau/MRL.tau);
          if (MRL.reference > 4095/2) {MRL.reference = 4095/2;}
      
          //MRL.reference = mapfloat(MRL.reference, 1400, 0, 4095/2, 0); //left motor: 0 fwd  4095 bwd  no current 2000
      
          MRL_outcmd(MRL.reference);
          MRR_outcmd(MRL.reference);
      
        }
        else {
        
          Current_F_filt = 4095/2;
          MRL.reference = 4095/2;
      
          MRL_outcmd(MRL.reference);
          MRR_outcmd(MRL.reference);
        }

      }
      else{
      ////////////////// MANUAL MODE /////////////////////
        ResetAllCL();
        
        // Panel Roll
        Roll_outcmd(AgriQJoyRx.VALUE_J1leftright); // Open loop incremental
      
        // Panel Pitch
        Pitch_outcmd(AgriQJoyRx.VALUE_J1updwn); // Open loop incremental control
        
        ////// Front Motors Controller
        MFront_ReferenceOmega(lambda, nu, vlong_lambda, yawrate_nu);
        
        PID_MFL = PID_Ctrl(time_sample*1000, MFL.reference, MFL.omega_measured,MFL.omega_measuredOLD , PID_MFL, -torque_max, torque_max); // ts in ms
        PID_MFL.ctrl = mapfloat(PID_MFL.ctrl, -torque_max, torque_max, 4095, 0); //left motor: 0 fwd  4095 bwd  no current 2000
        MFL_outcmd(PID_MFL.ctrl -100);
      
        PID_MFR = PID_Ctrl(time_sample, MFR.reference, MFR.omega_measured,MFR.omega_measuredOLD , PID_MFR, -torque_max, torque_max); // ts in s
        PID_MFR.ctrl = mapfloat(PID_MFR.ctrl, -torque_max, torque_max, 4095, 0); //left motor: 0 fwd  4095 bwd  no current 2000
        MFR_outcmd(PID_MFR.ctrl -120);

        ////// Rear Motor Controller
        if (AgriQJoyRx.VALUE_SW2 > 1600 && lambda > 0.1){
        
          Current_F_filt = 0.5*(PID_MFL.ctrl + PID_MFR.ctrl);
      
          MRL.reference = (1/distribution_ratio -1)*(Current_F_filt*MFL.tau/MRL.tau);
          if (MRL.reference > 4095/2) {MRL.reference = 4095/2;}
      
          MRL_outcmd(MRL.reference);
          MRR_outcmd(MRL.reference);
        }
        else {
        
        Current_F_filt = 4095/2;
        MRL.reference = 4095/2;
      
        MRL_outcmd(MRL.reference);
        MRR_outcmd(MRL.reference);
        }
      }  
  } // end of IF time sample


    ///// LOG
        
    //if (enb_DATALOG == 1)
    //{           
/*      
    if(millis() - time_oldLOG >= 300){
          time_oldLOG = millis();
          cli();
          
              Serial.print(lambda); Serial.print(";\t");
              Serial.print(Advance_reference); Serial.print(";\t");
              Serial.print(0.5*(MFL.odometry + MFR.odometry)); Serial.print("\n");
          
             Serial.print(micros()*1e-6); Serial.print(";");
              
              Serial.print(MFL.reference); Serial.print(";");
              Serial.print(MFL.omega_measured); Serial.print(";");
              Serial.print(MFL.current_measured); Serial.print(";");
              Serial.print(MFR.reference); Serial.print(";");
              Serial.print(MFR.omega_measured); Serial.print(";");
              Serial.print(MFR.current_measured); Serial.print(";");

              Serial.print(MRL.reference); Serial.print(";");
              Serial.print(MRL.omega_measured); Serial.print(";");
              Serial.print(MRL.current_measured); Serial.print(";");
              Serial.print(MRR.reference); Serial.print(";");
              Serial.print(MRR.omega_measured); Serial.print(";");
              Serial.print(MRR.current_measured); Serial.print(";");
                            
              Serial.print(Yaw_measured); Serial.print(";");
              Serial.print(Roll_measured); Serial.print(";");
              Serial.print(Pitch_measured); Serial.print("\n");
                          
          
              
              dataString = "";
              dataString += String(micros()*1e-6); // s          (1° colonna)
              dataString += ",";
              dataString += String(MFL.reference); // rad/s            (2° colonna)
              dataString += ",";  
              dataString += String(MFL.omega_measured); // rad/s            (3° colonna)
              dataString += ",";  
              dataString += String(PID_MFL.ctrl); // bit      (4° colonna)
              dataString += ",";  

              dataString += String(MFL.current_measured); // A      (5° colonna)
              dataString += ",";  
              
              dataString += String(MFR.reference); // rad/s    (6° colonna)
              dataString += ",";  
              dataString += String(MFR.omega_measured); // rad/s        (7° colonna)
              dataString += ",";  
              dataString += String(PID_MFR.ctrl); // bit    (8° colonna)
         
              dataString += ",";  
              dataString += String(MFR.current_measured); // A      (9° colonna)
              
             
              SDlog(dataString);
              Serial.println(dataString);
               
              sei();
          }
    
    } // end of IF LOG time sample
*/

  
}
// end of main loop
