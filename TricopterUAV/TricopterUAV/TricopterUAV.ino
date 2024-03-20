//========== Tricopter Program ==========

//Libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <QMC5883LCompass.h>
#include "Servo.h"

//********** compass HMC5883L **********
QMC5883LCompass compass;
int x, y, z;
float declinationAngle;
float heading_tilt = 0.0;
float headingDegrees;
float heading, setHeading;
float heading_reference;
float heading_control = 0.0;
float heading_filter = 0.0;
float heading_before = 0.0;
float acc_x, acc_y;
float compensateRoll, compensatePitch;
float cosComRoll, sinComRoll, cosComPitch, sinComPitch;
float Yh, Xh, Ymag_correct, Xmag_correct;
int normXAxis, normYAxis, normZAxis;
float yawDegrees=0.0;


//********** mpu 6050 **********
MPU6050 mpu;
MPU6050 accelgyro;
int16_t mx, my, mz;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float gyroX_filt, gyroY_filt, gyroZ_filt;

float G_Dt = 0.005;

double rad_yaw, rad_pitch, rad_roll;
double roll_kalman, pitch_kalman, yaw_kalman;
double accum_roll = 0;
double accum_pitch = 0;
double k_acc = 0;
double k_gps = 0;
double yaw_deg;
double pitch_deg;
double roll_deg;
double pitch_deg_previous, roll_deg_previous;
float gyro_roll_input  = 0.0;
float gyro_pitch_input = 0.0;
float gyro_yaw_input  = 0.0;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_ACCELGYRO

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
boolean interruptLock = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}

//********** brushless motor **********
#define MOTOR1 2
#define MOTOR2 3
#define MOTOR3 4

Servo brushless1;
Servo brushless2;
Servo brushless3;

int motor1;
int motor2;
int motor3;

//********** servo motor **********
Servo myservo;
int servoAngleInit = 45;
int servo;
float yawControlServo;

//********** remote channels **********
volatile int channel1 = 0;
volatile int roll_channel = 0;
volatile int channel2 = 0;
volatile int throttle_channel = 0;
volatile int channel3 = 0;
volatile int pitch_channel = 0;
volatile int channel4 = 0;
volatile int yaw_channel = 0;
volatile int channel5 = 0;
volatile int ch5_channel = 0;

//********** inputs variable **********
int ch5, throttle, throttle_input, heading_mode, heading_mode1;
int roll_input, pitch_input, yaw_input;
unsigned long timeProgram, previousTimeProgram;
unsigned long timeServo, previousTimeServo;
unsigned long deltaTime_control_rate, lastTime_control_rate;
char inChar;

//********** controller **********
float gyroRoll, gyroPitch, gyroYaw;
float rollSetpoint, pitchSetpoint, yawSetpoint;
float rollLevel, pitchLevel;
float errorRoll, errorPitch, errorYaw;
float ControllerIRoll, ControllerDRoll, ControllerDRollLast;
float ControllerIPitch, ControllerDPitch, ControllerDPitchLast;
float ControllerIYaw, ControllerDYaw, ControllerDYawLast;
float ControlRoll, ControlPitch, ControlYaw;
float roll_rate_input    = 0.0;
float pitch_rate_input   = 0.0;
float roll_level_adjust  = 0.0;
float pitch_level_adjust = 0.0;
float GainProll  = 0.0;
float GainIroll  = 0.0;
float GainDroll  = 0.0;
float GainPpitch = 0;
float GainIpitch = 0;
float GainDpitch = 0;
float GainPyaw   = 0;
float GainIyaw   = 0;
float GainDyaw   = 0;
int start;
int maxController = 200;

int rpm_motor1 = 0;
int rpm_motor2 = 0;
int rpm_motor3 = 0;

float kt = 0.000042302;
float kd = 0.00000377696;
float d  = 0.0185;
float d1 = 0.925;
float d2 = 0.160;

double q1 = 0.0;
double q2 = 0.0;
double q3 = 0.0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial3.begin(57600);

  //sensors
  init_MPU();
  compass.init();

  //remote signals
  remote_init();
  delay(50);

  //actuators
  myservo.attach(5);
  servo_setup();
  motor_setup();
}

void loop()
{
  get_YPR();
  compass_update();
  mapremote();

//  gyro_roll_input  = (gyro_roll_input *0.7)+(gx*0.3);
//  gyro_pitch_input = (gyro_pitch_input*0.7)+(gy*0.3);
//  gyro_yaw_input   = (gyro_yaw_input  *0.7)+(gz*0.3);

  control_update();

  serialEvent();
  timeProgram = micros();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    Serial.print("sudut roll = ")           ; Serial.print(roll_deg)            ; Serial.print("  ");
//    Serial.print("sudut pitch = ")          ; Serial.print(pitch_deg)           ; Serial.print("  ");
//    Serial.print("headingDegrees = ")       ; Serial.print(headingDegrees)      ; Serial.print("  ");
//    Serial.print(" yawDegrees = ")          ; Serial.print(heading_control)     ; Serial.print("  ");
    
    Serial.print("mtr1= ")                  ; Serial.print(motor1)              ; Serial.print("  ");
    Serial.print("mtr2= ")                  ; Serial.print(motor2)              ; Serial.print("  ");
//    Serial.print("mtr3= ")                  ; Serial.print(motor3)              ; Serial.print("  ");

    Serial.print("ControlRoll= ")           ; Serial.print(ControlRoll)         ; Serial.print("  ");
//    Serial.print("ControlPitch= ")          ; Serial.print(ControlPitch)        ; Serial.print("  ");
//    Serial.print("ControlYaw=  ")           ; Serial.print(ControlYaw)          ; Serial.print("  ");
    
    Serial.print("P= ")                     ; Serial.print(GainProll)           ; Serial.print("  ");
    Serial.print("I= ")                     ; Serial.print(GainIroll)           ; Serial.print("  ");
    Serial.print("D=  ")                    ; Serial.print(GainDroll)          ; Serial.print("  ");
//    Serial.print("I= ")                     ; Serial.print(GainIpitch)          ; Serial.print("  ");
//    Serial.print("D=  ")                    ; Serial.print(GainDpitch)          ; Serial.print("  ");

    Serial.print("roll_input=  ")           ; Serial.print(roll_input)          ; Serial.print("  ");
//    Serial.print("pitch_input=  ")          ; Serial.print(pitch_input)         ; Serial.print("  ");
//    Serial.print("yaw_input=  ")            ; Serial.print(yaw_input)           ; Serial.print("  ");
//    Serial.print("throttle_input=  ")       ; Serial.print(throttle_input)      ; Serial.print("  ");

//    Serial.print("q1=  ")                   ; Serial.print(q1)                  ; Serial.print("  ");
//    Serial.print("q2=  ")                   ; Serial.print(q2)                  ; Serial.print("  ");
//    Serial.print("q3=  ")                   ; Serial.print(q3)                  ; Serial.print("  ");

//    Serial.print("servo  ")                 ; Serial.print(servoAngleInit)      ; Serial.print("  ");
//    Serial.print(" yawDegrees = ")          ; Serial.print(heading_control)     ; Serial.print("  ");
//    Serial.print("ch 1 = ");        Serial.print(roll_channel);      Serial.print(" ");
//    Serial.print("ch 2 = ");        Serial.print(pitch_channel);     Serial.print(" ");
//    Serial.print("ch 3 = ");        Serial.print(throttle_channel);  Serial.print(" ");
//    Serial.print("ch 4 = ");        Serial.print(yaw_channel);       Serial.print(" ");
    Serial.print("ch 5 = ");        Serial.print(ch5_channel);       Serial.print(" ");
    Serial.print("roll_rate_input = ");        Serial.print(roll_rate_input);       Serial.print(" ");
    Serial.println();
    previousTimeProgram = micros();
  }
}
