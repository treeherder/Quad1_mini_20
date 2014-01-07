/* ********************************************************************** */
/*                    JJ ArduIMU Quadcopter                               */
/*                                                                        */
/* Code based on ArduIMU DCM code from Diydrones.com                      */
/* Date : 22-02-2010                                                      */
/* Author : Jose Julio                                                    */
/* Version : 1.20 (mini)                                                  */
/* This version supports the optional magnetometer                        */
/* With the magnetometer we eliminate the yaw drift                       */
/* ********************************************************************** */

#include <inttypes.h>
#include <math.h>
#include <Wire.h>   // For magnetometer readings

// QuadCopter PID GAINS
#define KP_QUAD_ROLL 1.8 // 2.2   //1.75
#define KD_QUAD_ROLL 0.42 // 0.54  //0.45
#define KI_QUAD_ROLL 0.5 // 0.45  //0.5
#define KP_QUAD_PITCH 1.8 // 2.2   //1.75
#define KD_QUAD_PITCH 0.42 // 0.54  //0.45
#define KI_QUAD_PITCH 0.5 // 0.45  //0.5
#define KP_QUAD_YAW 3.8 // 4.6  //3.2 //2.6
#define KD_QUAD_YAW 1.3 // 0.7  //0.8 //0.4
#define KI_QUAD_YAW 0.15 // 0.2  //0.15

#define KD_QUAD_COMMAND_PART 18.0   // for special KD implementation (in two parts)

// The IMU should be correctly adjusted : Gyro Gains and also initial IMU offsets:
// We have to take this values with the IMU flat (0º roll, 0ºpitch)
#define acc_offset_x 508 
#define acc_offset_y 504
#define acc_offset_z 501       // We need to rotate the IMU exactly 90º to take this value  
#define gyro_offset_roll 370  
#define gyro_offset_pitch 373
#define gyro_offset_yaw 380

// We need to now the number of channels to read from radio
// If this value is not well adjusted you will have bad radio readings... (you can check the radio at the begining of the setup process)
#define MAX_CHANNELS    7       // Number of radio channels to read (7 is the number if you use the PPM encoder from store.diydrones.com)

#define MIN_THROTTLE 1037       // Throttle pulse width at minimun...
#define CHANN_CENTER 1500

#define SPEKTRUM 1  // Spektrum radio

/* ***************************************************************************************** */

// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/º/s, 3.22mV/ADC step => 1.03
// Tested values : 0.96,0.96,0.94
#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.94 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.0125  //0.010 // Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.000010 // Pitch&Roll Integrator Gain
#define Kp_YAW 1.2 // Yaw Porportional Gain  
#define Ki_YAW 0.00005 // Yaw Integrator Gain

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 3 // >1 use min speed filter for yaw drift cancellation, 0=do not use

/*For debugging purposes*/
#define OUTPUTMODE 1  //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data

uint8_t sensors[6] = {6,7,3,0,1,2};  // For Hardware v2 flat

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
int SENSOR_SIGN[]={1,-1,-1,1,-1,1,-1,-1,-1}; //{1,-1,-1,-1,1,-1}

float AN[6]; //array that store the 6 ADC filtered data
float AN_OFFSET[6]; //Array that stores the Offset of the gyros

float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_magnitude;
float Accel_weight;
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=0;
float COGX=0; //Course overground X axis
float COGY=1;

float roll=0;
float pitch=0;
float yaw=0;

//Magnetometer variables
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;

unsigned int counter=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

//#define roll_offset 0.0
//#define pitch_offset 0.0 

// PPM Rx signal read (ICP) constants and variables
#define Servo1Pin  9            // Servo pins...
#define Servo2Pin 10
#define icpPin	    8           // Input Capture Pin (Rx signal reading)
//#define MAX_CHANNELS    4       // Number of radio channels to read
#define SYNC_GAP_LEN    8000    // we assume a space at least 4000us is sync (note clock counts in 0.5 us ticks)
#define MIN_IN_PULSE_WIDTH (750)  //a valid pulse must be at least 750us (note clock counts in 0.5 us ticks)
#define MAX_IN_PULSE_WIDTH (2250) //a valid pulse must be less than  2250us
static volatile unsigned int Pulses[ MAX_CHANNELS + 1];  // Pulse width array
static volatile uint8_t  Rx_ch = 0;
int Neutro[MAX_CHANNELS+1];    // Valores para los neutros en caso de fallos...
byte radio_status=0;           // radio_status = 1 => OK, 0 => No Radio signal
static volatile unsigned int ICR1_old = 0;
static volatile unsigned int Timer1_last_value;  // to store the last timer1 value before a reset
int ch1;    // Channel values
int ch2;
int ch3;
int ch4;
int ch_aux;
int ch_aux2;
int ch1_old;    // Channel values
int ch2_old;
int ch3_old;
int ch4_old;
int ch_aux_old;
int ch_aux2_old;


// PPM Rx signal read END

// Servo Timer2 variables (Servo Timer2)
#define SERVO_MAX_PULSE_WIDTH 2000
#define SERVO_MIN_PULSE_WIDTH 900
#define SERVO_TIMER2_NUMSERVOS 4         // Put here the number of servos. In this case 4 ESC´s
typedef struct {
  uint8_t pin;
  int value;
  uint8_t counter;
}  servo_t;
uint8_t num_servos=SERVO_TIMER2_NUMSERVOS;
servo_t Servos[SERVO_TIMER2_NUMSERVOS];
static volatile uint8_t Servo_Channel;
static volatile uint8_t ISRCount=0;
static volatile unsigned int Servo_Timer2_timer1_start;
static volatile unsigned int Servo_Timer2_timer1_stop;
static volatile unsigned int Servo_Timer2_pulse_length;
// Servo Timer2 variables END 

// Servo variables (OC1 and OC2) for standard servos [disabled in this version]
unsigned int Servo1;
unsigned int Servo2;


//GPS variable definition
union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

// Flight GPS variables
int gpsFix=1; //This variable store the status of the GPS
float lat=0; // store the Latitude from the gps
float lon=0;// Store guess what?
float alt_MSL=0; //This is the alt.
long iTOW=0; //GPS Millisecond Time of Week
long alt=0; //Height above Ellipsoid 
float speed_3d=0; //Speed (3-D)
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=90;//This is the runaway direction of you "plane" in degrees
char data_update_event=0;
byte GPS_Error=1;

// GPS UBLOX
byte ck_a=0;     // Packet checksum
byte ck_b=0;
byte UBX_step=0;
byte UBX_class=0;
byte UBX_id=0;
byte UBX_payload_length_hi=0;
byte UBX_payload_length_lo=0;
byte UBX_payload_counter=0;
byte UBX_buffer[40];
byte UBX_ck_a=0;
byte UBX_ck_b=0;


// Navigation control variables
//float alt_error;
//float course_error;
//float course_error_old;

// ADC variables
volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile uint16_t analog_buffer[8];
volatile uint8_t analog_count[8];
int an_count;

// Attitude control variables
float comando_rx_roll=0;        // comandos recibidos rx
float comando_rx_roll_old;
float comando_rx_roll_diff;
float comando_rx_pitch=0;
float comando_rx_pitch_old;
float comando_rx_pitch_diff;
float comando_rx_yaw=0;
float comando_rx_yaw_diff;
int control_roll;           // resultados del control
int control_pitch;
int control_yaw;
float K_aux;
float roll_I=0;
float roll_D;
float err_roll;
float err_roll_ant;
float pitch_I=0;
float pitch_D;
float err_pitch;
float err_pitch_ant;
float yaw_I=0;
float yaw_D;
float err_yaw;
float err_yaw_ant;

// AP_mode : 1=> Radio Priority(manual mode) 2=>Stabilization assist mode 3=>Autopilot mode
byte AP_mode = 2;             

long t0;
int num_iter;
float aux_debug;

#define MAX_ROLL_ANGLE 18 
#define MAX_PITCH_ANGLE 18


/* ************************************************************ */
// ROLL, PITCH and YAW PID controls...
void Attitude_control(){

  // ROLL CONTROL    
  err_roll_ant = err_roll;
  if (AP_mode==2)        // Stabilization mode
    err_roll = comando_rx_roll - ToDeg(roll);
  else
    err_roll = 0;
  err_roll = constrain(err_roll,-30,30);  // to limit max roll command...
  
  roll_I += err_roll*G_Dt;
  roll_I = constrain(roll_I,-50,50);
  // D term implementation => two parts: gyro part and command part
  // To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
  // Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
  // We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs
  roll_D = comando_rx_roll_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[0]);  // Take into account Angular velocity of the stick (command)
  
  // PID control
  control_roll = KP_QUAD_ROLL*err_roll + KD_QUAD_ROLL*roll_D + KI_QUAD_ROLL*roll_I; 
  
  // PITCH CONTROL
  err_pitch_ant = err_pitch;
  if (AP_mode==2)        // Stabilization mode
    err_pitch = comando_rx_pitch - ToDeg(pitch);
  else
    err_pitch = 0;
  err_pitch = constrain(err_pitch,-30,30);  // to limit max pitch command...
  
  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-50,50);
  // D term
  pitch_D = comando_rx_pitch_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[1]);
 
  // PID control
  control_pitch = KP_QUAD_PITCH*err_pitch + KD_QUAD_PITCH*pitch_D + KI_QUAD_PITCH*pitch_I; 
  
  // YAW CONTROL
  err_yaw_ant = err_yaw;
  if (AP_mode==2){        // Stabilization mode
    err_yaw = comando_rx_yaw - ToDeg(yaw);
    if (err_yaw > 180)    // Normalize to -180,180
      err_yaw -= 360;
    else if(err_yaw < -180)
      err_yaw += 360;
    }
  else
    err_yaw = 0;
  err_yaw = constrain(err_yaw,-60,60);  // to limit max yaw command...
  
  yaw_I += err_yaw*G_Dt;
  yaw_I = constrain(yaw_I,-50,50);
  yaw_D = comando_rx_yaw_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[2]);
 
  // PID control
  control_yaw = KP_QUAD_YAW*err_yaw + KD_QUAD_YAW*yaw_D + KI_QUAD_YAW*yaw_I;
}


int channel_filter(int ch, int ch_old)
{
  int diff_ch_old;
  int result;
  
  result = ch;
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
     {
     if (diff_ch_old<-30)
       result = ch_old-30;        // We limit the max difference between readings
     }
  else
     {
     if ((diff_ch_old>30)&&(ch_old>900))    // We take into account that ch_old could be 0 (not initialized)
       result = ch_old+30;
     }
  return((ch+ch_old)/2);     // small filtering
}


long timer=0; //general porpuse timer 
long timer_old;

void setup(){
  
  int i;
  int aux;
 
  Serial.begin(38400);
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
  pinMode(8,INPUT);    // Rx Radio Input
  pinMode(5,OUTPUT);   // Red LED
  pinMode(6,OUTPUT);   // BLue LED
  pinMode(7,OUTPUT);   // Yellow LED
  pinMode(9,OUTPUT);   // Servo1
  pinMode(10,OUTPUT);  // Servo2  Right Motor
  pinMode(11,OUTPUT);  // Servo3  Left Motor
  pinMode(12,OUTPUT);  // Servo4  Front Motor
  pinMode(13,OUTPUT);  // Servo5  Back Motor
 
  ch1=MIN_THROTTLE;
  ch2=MIN_THROTTLE;
  ch3=MIN_THROTTLE;
  ch4=MIN_THROTTLE;
 
  delay(100);
  comando_rx_yaw = 0;
  Servo1 = 1500;
  Servo2 = 1500;
  Serial.println();
  Serial.println("JJ ArduIMU Quadcopter 1.20 mini PID");
  RxServoInput_ini();
  delay(1000);
   
 // Take neutral radio values...
 Neutro[1] = RxGetChannelPulseWidth(1);
 Neutro[2] = RxGetChannelPulseWidth(2);
 Neutro[3] = RxGetChannelPulseWidth(3);
 Neutro[4] = RxGetChannelPulseWidth(4);
 Neutro[5] = RxGetChannelPulseWidth(5);
 Neutro[6] = RxGetChannelPulseWidth(6);
 for (i=0; i<80; i++)
   {
   Neutro[1] = (Neutro[1]*0.8 + RxGetChannelPulseWidth(1)*0.2);
   Neutro[2] = (Neutro[2]*0.8 + RxGetChannelPulseWidth(2)*0.2);
   Neutro[3] = (Neutro[3]*0.8 + RxGetChannelPulseWidth(3)*0.2);
   Neutro[4] = (Neutro[4]*0.8 + RxGetChannelPulseWidth(4)*0.2);
   Neutro[5] = (Neutro[5]*0.8 + RxGetChannelPulseWidth(5)*0.2);
   Neutro[6] = (Neutro[6]*0.8 + RxGetChannelPulseWidth(6)*0.2);
   delay(25);
   }
 
 Serial.print("Rx values: ");
 Serial.print(Neutro[1]);
 Serial.print(",");
 Serial.print(Neutro[2]);
 Serial.print(",");
 Serial.print(Neutro[3]);
 Serial.print(",");
 Serial.println(Neutro[4]);
 Serial.print(",");
 Serial.println(Neutro[5]);
 Serial.print(",");
 Serial.println(Neutro[6]);
 
 // Roll, Pitch and Throttle have fixed neutral values (the user can trim the radio)
 #if SPEKTRUM==1
   Neutro[3] = CHANN_CENTER;
   Neutro[2] = CHANN_CENTER;
   Neutro[1] = MIN_THROTTLE;
 #else
   Neutro[1] = CHANN_CENTER;
   Neutro[2] = CHANN_CENTER;
   Neutro[3] = MIN_THROTTLE;
 #endif
 
 // Assign pins to servos
 num_servos = 4;
 Servos[0].pin = 10;       // Left motor
 Servos[1].pin = 11;       // Right motor
 Servos[2].pin = 12;       // Front motor
 Servos[3].pin = 13;       // Back motor
 Servo_Timer2_set(0,MIN_THROTTLE);   // First assign values to servos
 Servo_Timer2_set(1,MIN_THROTTLE);
 Servo_Timer2_set(2,MIN_THROTTLE);
 Servo_Timer2_set(3,MIN_THROTTLE);
 Servo_Timer2_ini();              // Servo Interrupt initialization
 
 Analog_Reference(EXTERNAL);
 Analog_Init();
 I2C_Init();
 delay(100);
 
 // Magnetometer initialization
  Compass_Init();
 
 Read_adc_raw();
 delay(20);
 
 // Offset values for accels and gyros...
 AN_OFFSET[3] = acc_offset_x;
 AN_OFFSET[4] = acc_offset_y;
 AN_OFFSET[5] = acc_offset_z;
 AN_OFFSET[0] = gyro_offset_roll;
 AN_OFFSET[1] = gyro_offset_pitch;
 AN_OFFSET[2] = gyro_offset_yaw;
 
 // Take the gyro offset values
 for(int i=0;i<600;i++)
    {
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for offset.
      AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
    delay(20);
    }
 
 Serial.print("AN[0]:");
 Serial.println(AN_OFFSET[0]);
 Serial.print("AN[1]:");
 Serial.println(AN_OFFSET[1]);
 Serial.print("AN[2]:");
 Serial.println(AN_OFFSET[2]);
 Serial.print("AN[3]:");
 Serial.println(AN_OFFSET[3]);
 Serial.print("AN[4]:");
 Serial.println(AN_OFFSET[4]);
 Serial.print("AN[5]:");
 Serial.println(AN_OFFSET[5]);

 delay(1500);
 
 // Wait until throttle stick is at bottom
 #if SPEKTRUM==1
 while (RxGetChannelPulseWidth(1)>(MIN_THROTTLE+50)){
 #else
 while (RxGetChannelPulseWidth(3)>(MIN_THROTTLE+50)){
 #endif
   //Serial.println("Move throttle stick to bottom to start !!!");
   Serial.print("Radio Channels:");
   for (i=1;i<=MAX_CHANNELS;i++)
     {
     Serial.print(RxGetChannelPulseWidth(i));
     Serial.print(",");
     }
   Serial.println();
   delay(100);
 }

 Read_adc_raw();   // Start ADC readings...
 timer = millis();
 delay(20);
 
 digitalWrite(7,HIGH);
}

void loop(){
  
  int aux;
  float aux_float;
  
  if((millis()-timer)>=14)   // 14ms => 70 Hz loop rate 
  {
    counter++;
    timer_old = timer;
    timer=millis();
    G_Dt = (timer-timer_old)/1000.0;      // Real time of loop run 
    num_iter++;
	
    // IMU DCM Algorithm
    Read_adc_raw();
    
    if (counter > 7)  // Read compass data at 10Hz... (7 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // *****************
    
    // Telemetry data...
    
    aux = ToDeg(roll)*10;
    Serial.print(aux);
    Serial.print(",");
    aux = (ToDeg(pitch))*10;
    Serial.print(aux);
    Serial.print(",");
    aux = ToDeg(yaw)*10;
    Serial.print(aux);
    /*
    Serial.print(magnetom_x);
    Serial.print(",");
    Serial.print(magnetom_y);
    Serial.print(",");
    Serial.println(magnetom_z);
    */
    
    if (radio_status == 1){
      radio_status=2;   // Radio frame read
      ch1_old = ch1;
      ch2_old = ch2;
      ch3_old = ch3;
      ch4_old = ch4;
      ch_aux_old = ch_aux;
      ch_aux2_old = ch_aux2;
      #if SPEKTRUM==1
        ch1 = channel_filter(RxGetChannelPulseWidth(2),ch1_old);   // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(3),ch2_old);   // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(1),ch3_old);   // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4_old);   // Ruder
        ch_aux = channel_filter(RxGetChannelPulseWidth(6),ch_aux_old);  // Aux
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(5),ch_aux2_old);  // Aux2
      #else
        ch1 = RxGetChannelPulseWidth(1);   // Read radio channel values
        ch2 = RxGetChannelPulseWidth(2);
        ch3 = RxGetChannelPulseWidth(3);
        ch4 = RxGetChannelPulseWidth(4);
        //ch_aux = RxGetChannelPulseWidth(6);  // Aux
        //ch_aux2 = RxGetChannelPulseWidth(5);  // Aux
      #endif      
      
      // Commands from radio Rx... 
      // Stick position defines the desired angle in roll, pitch and yaw
      comando_rx_roll_old = comando_rx_roll;
      comando_rx_roll = (ch1-CHANN_CENTER)/15.0;
      comando_rx_roll_diff = comando_rx_roll-comando_rx_roll_old;
      comando_rx_pitch_old = comando_rx_pitch;
      comando_rx_pitch = (ch2-CHANN_CENTER)/15.0;
      comando_rx_pitch_diff = comando_rx_pitch-comando_rx_pitch_old;
      aux_float = (ch4-Neutro[4])/200.0;
      comando_rx_yaw += aux_float;
      comando_rx_yaw_diff = aux_float;
      if (comando_rx_yaw > 180)         // Normalize yaw to -180,180 degrees
        comando_rx_yaw -= 360.0;
      else if (comando_rx_yaw < -180)
        comando_rx_yaw += 360.0;
      
      // I use K_aux to adjust gains linked to a knob in the radio... [not used now]
      K_aux = K_aux*0.8 + ((ch_aux-1500)/100.0 + 0.6)*0.2;
      if (K_aux < 0)
        K_aux = 0;
   
      AP_mode = 2;   // Stabilization assist mode      
      }
    else if (radio_status==0)
      {  // Radio_status = 0 Lost radio signal => Descend...
      ch3--;   // Descend  (Reduce throttle)
      if (ch3<MIN_THROTTLE)
        ch3 = MIN_THROTTLE;
      comando_rx_roll = 0;     // Stabilize to roll=0, pitch=0, yaw not important here
      comando_rx_pitch = 0;
      Attitude_control();
      // Quadcopter mix
      Servo_Timer2_set(0,ch3 - control_roll - control_yaw);    // Right motor
      Servo_Timer2_set(1,ch3 + control_roll - control_yaw);    // Left motor
      Servo_Timer2_set(2,ch3 + control_pitch + control_yaw);   // Front motor
      Servo_Timer2_set(3,ch3 - control_pitch + control_yaw);   // Back motor
      }  
  
    // Attitude control 
    //comando_rx_yaw = 0;
    Attitude_control();
      
    // Quadcopter mix
    if (ch3 > (MIN_THROTTLE+40))  // Minimun throttle to start control
      {
      Servo_Timer2_set(0,ch3 - control_roll - control_yaw);    // Right motor
      Servo_Timer2_set(1,ch3 + control_roll - control_yaw);    // Left motor
      Servo_Timer2_set(2,ch3 + control_pitch + control_yaw);   // Front motor
      Servo_Timer2_set(3,ch3 - control_pitch + control_yaw);   // Back motor
      }
    else
      {
      roll_I = 0;  // reset I terms...
      pitch_I = 0;
      yaw_I = 0; 
      Servo_Timer2_set(0,MIN_THROTTLE);  // Motors stoped
      Servo_Timer2_set(1,MIN_THROTTLE);
      Servo_Timer2_set(2,MIN_THROTTLE);
      Servo_Timer2_set(3,MIN_THROTTLE);
      // Initialize yaw command to actual yaw
      comando_rx_yaw = ToDeg(yaw);
      comando_rx_yaw_diff = 0;
      }
    
    
    Serial.print(",");
    Serial.print(K_aux);
    Serial.print(",");
    Serial.print(timer-timer_old); // G_Dt*1000
            
    /*    
    Serial.print(",");
    Serial.print(ch1);
    Serial.print(",");
    Serial.print(ch2);
    Serial.print(",");
    Serial.print(ch3);
    Serial.print(",");
    Serial.print(ch4);
    */
 
    Serial.println();  // Line END
    }
}

