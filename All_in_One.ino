#define RESTRICT_PITCH 
#define _eTaskGetState
#define MIN_PULSE_LENGTH 900
#define MAX_PULSE_LENGTH 1900
#include "mpu6050.h"
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "filters.h"
#include <Wire.h>
#include <Kalman.h>
#include <esp_now.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <ESP32_Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
//Declare intances 
MPU6050 accelgyro;
HMC5883L mag(0x1E);
MS5611 MS5611(0x77);

//Declare for comple filter
const float cutoff_freq = 10.0;
const float sample_time = 0.005;
IIR::ORDER  order  = IIR::ORDER::OD2;
Filter f(cutoff_freq, sample_time, order);

// Heading variables
int16_t mx, my, mz;
float heading, declinationAngle, headingdeg;

//Declare Kalman 
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double Timer;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
uint8_t devStatus;   
//Task Handle
TaskHandle_t Task1,Task2,Task3,Task4;

//MS5611 Vars
double ref_pres,altitude_r,filteredval;

// MAC Address
uint8_t broadcastAddress[] = {0xB0, 0xB2, 0x1C, 0xA8, 0x77, 0xB0};

//Structure for data transfer
typedef struct struct_message {
  char a[32];
  float b;
  float c;
  float d;
  double e;
  int f;
  int g;
  int h;
  int i;
  int l;
  int m;
} struct_message;
esp_now_peer_info_t peerInfo;
struct_message myData;

//Data Received
String Data_recv;
char dat;
typedef struct delivery
{
  String A;
} delivery;
delivery deli_package;

//Other Variables
String Data,height,rev_Pid;
int strlength,dex;

//Declare PID Lib
double Kp_1=40,Ki_1=1.0,Kd_1=0.1;
double Kp_2=10,Ki_2=0.1,Kd_2=0.1;
double Kp_3=10,Ki_3=0.1,Kd_3=0.1;
double Kp_4=5,Ki_4=0,Kd_4=1.0;
double input_1,out_Thrust,input_2,out_Roll,input_3,out_Pitch,input_4,out_Yaw;
double setpoint_1,setpoint_2,setpoint_3,setpoint_4;
PID pid_Thrust(&input_1, &out_Thrust, &setpoint_1, Kp_1, Ki_1, Kd_1, DIRECT);
PID pid_Roll(&input_2, &out_Roll, &setpoint_2, Kp_2, Ki_2, Kd_2, DIRECT);
PID pid_Pitch(&input_3, &out_Pitch, &setpoint_3, Kp_3, Ki_3, Kd_3, DIRECT);
PID pid_Yaw(&input_4, &out_Yaw, &setpoint_4, Kp_4, Ki_4, Kd_4, DIRECT);

//Declare motor vars
double motor_1,motor_2,motor_3,motor_4,motor_5,motor_6;

//Declare servo lib - output for motors
Servo servo1,servo2,servo3,servo4,servo5,servo6;

const int servo1Pin = 18;
const int servo2Pin = 5;
const int servo3Pin = 17;
const int servo4Pin = 16;
const int servo5Pin = 4;
const int servo6Pin = 15;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);
    Serial.print("[2J");  
    accelgyro.initialize();
    accelgyro.setI2CBypassEnabled(true);
    Serial.begin(115200);

  if (!MS5611.begin()) {
    Serial.println("MS5611 not found, check wiring!");
    while (1);
  } 

    WiFi.mode(WIFI_STA); //WIFI mode
    if (esp_now_init() != ESP_OK) //initiliza ESP-NOW
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);  
    esp_now_register_recv_cb(OnDataRecv);
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    // initialize device
  mag.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  MS5611.setOversampling(OSR_STANDARD);
  delay(1000);
  //devStatus = accelgyro.dmpInitialize();
  Serial.println("Calibrating MPU6050");
  accelgyro.CalibrateAccel(6);
  accelgyro.CalibrateGyro(6);
  delay(1000);
  
  // First angle or offsets
  accelgyro.getAcceleration(&ax,&ay,&az);
  accX = (int16_t) ax;
  accY = (int16_t) ay;
  accZ = (int16_t) az;
 
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  //Setpoint
  setpoint_2 = 0.1;
  setpoint_3 = 0.1;

  // Get Setpoint height
  MS5611.read();
  ref_pres = MS5611.getPressure();
  altitude_r = MS5611.getAltitude(ref_pres,1013.25);
  // filteredval = f.filterIn(altitude_r);
  setpoint_1 = (int)altitude_r+1.5;
  delay(1000);
  //PID setup
  pid_Thrust.SetMode(AUTOMATIC);
  pid_Thrust.SetOutputLimits(-1900, 1900);
  pid_Roll.SetMode(AUTOMATIC);
  pid_Roll.SetOutputLimits(-1900, 1900);
  pid_Pitch.SetMode(AUTOMATIC);
  pid_Pitch.SetOutputLimits(-1900, 1900);
  pid_Yaw.SetMode(AUTOMATIC);
  pid_Yaw.SetOutputLimits(-1900, 1900);
  //
  servo1.attach(servo1Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  servo2.attach(servo2Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  servo3.attach(servo3Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  servo4.attach(servo4Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  servo5.attach(servo5Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  servo6.attach(servo6Pin,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
  // Setpoint heading
  setpoint_4 = 33;
  delay(1000);
  //Declare first task on core0
  xTaskCreatePinnedToCore
 ( 
    Task1core,
    "Task1",
    2048,
    NULL,
    1,
    &Task1,
    0    
 );
 vTaskSuspend(Task1);
 Serial.println("Task1 is created and suspended");
 delay(500);
 //Declare second task on core1
  xTaskCreatePinnedToCore
 ( 
    Task2core,
    "Task2",
    1024,
    NULL,
    1,
    &Task2,
    1    
 );
 vTaskSuspend(Task2);
 Serial.println("Task2 is created and suspended");
 delay(500);
 xTaskCreatePinnedToCore
 (
   TaskCore1Pid,
   "TaskPID",
   1024,
   NULL,
    2,
    &Task3,
    1   
 );
 vTaskSuspend(Task3);
 Serial.println("Task3 is created and suspended");
 delay(500);
  xTaskCreatePinnedToCore
 (
   TaskCore1BLDC,
   "TaskBLDC",
   1024,
   NULL,
   2,
    &Task4,
    0   
 );
 vTaskSuspend(Task4);
 Serial.println("Task BLDC Calibration is created and suspended");
 delay(500);
}

void loop() 
{
}
void Task1core(void * parameter)
{
  for(;;)
  {
    MS5611.read();
    ref_pres = MS5611.getPressure();
    altitude_r = MS5611.getAltitude(ref_pres,1013.25);
    filteredval = f.filterIn(altitude_r);
    mag.getHeading(&mx, &my, &mz);
    heading = atan2(my , mx );
    declinationAngle = (0 - (46.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    headingdeg = heading * 180/M_PI;
    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    accX = (int16_t) ax;
    accY = (int16_t) ay;
    accZ = (int16_t) az;
    gyroX = (int16_t) gx;
    gyroY = (int16_t) gy;
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    // gyroYangle += kalmanY.getRate() * dt;

    // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    // compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    // if(kalAngleX > 20 || kalAngleY > 20) 
    // {vTaskSuspend(Task3);
    // allstop();
    // }
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }
}
void Task2core (void * parameter)
{
    for(;;)
    {
        //strcpy(myData.a, "Gyro Data");
        myData.c = kalAngleX;
        myData.b = kalAngleY;
        myData.d = headingdeg;
        myData.e = filteredval;
        myData.f = (int)motor_1;
        myData.g = (int)motor_2;
        myData.h = (int)motor_3;
        myData.i = (int)motor_4;
        myData.l = (int)motor_5;
        myData.m = (int)motor_6;
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        // if (result == ESP_OK) {
        // Serial.println("Sending confirmed");
        // }
        // else 
        // {
        // Serial.println("Sending error");
        // }

        
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}

void TaskCore1Pid (void* parameter)
{
  for(;;)
  {
    input_1 = (int)filteredval;
    input_2 = kalAngleY;
    input_3 = kalAngleX;
    //input_4 = headingdeg;
    //pid_Thrust.Compute();
    pid_Roll.Compute();
    pid_Pitch.Compute();
    //pid_Yaw.Compute();
    motor_1 = out_Thrust + out_Roll + out_Pitch; //front left
    motor_2 = out_Thrust + out_Roll + out_Pitch; //rear left
    motor_3 = out_Thrust + out_Roll - out_Pitch; //back left
    motor_4 = out_Thrust - out_Roll - out_Pitch; // back right
    motor_5 = out_Thrust - out_Roll + out_Pitch; //rear righ
    motor_6 = out_Thrust - out_Roll + out_Pitch; //front right
    // esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //pulse for esc 1 (front-right - CCW) motor 6
    // esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //pulse for esc 2 (rear-right - CW)
    // esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //pulse for esc 3 (rear-left - CCW)
    // esc_3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //pulse for esc 4 (front-left - CW) motor 1
    // esc_5 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //?
    // esc_6 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //?
    //for 4 motors
    // motor_1 = output_1+output_2+output_3;
    // motor_2 = output_1-output_2+output_3;
    // motor_3 = output_1+output_2-output_3+output_4;
    // motor_4 = output_1-output_2-output_3;
    // motor_5 = output_1+output_2-output_3;
    // motor_6 = output_1-output_2+output_3-output_4;
    servo1.writeMicroseconds(motor_1 = motor_1*10 < 950 ? 950 :motor_1*10 > 1200 ? 1200 : motor_1*10);
    servo2.writeMicroseconds(motor_2 = motor_2*10 < 950 ? 950 :motor_2*10 > 1200 ? 1200 : motor_2*10);
    servo3.writeMicroseconds(motor_3 = motor_3*10 < 950 ? 950 :motor_3*10 > 1200 ? 1200 : motor_3*10);
    servo4.writeMicroseconds(motor_4 = motor_4*10 < 950 ? 950 :motor_4*10 > 1200 ? 1200 : motor_4*10);
    servo5.writeMicroseconds(motor_5 = motor_5*10 < 950 ? 950 :motor_5*10 > 1200 ? 1200 : motor_5*10);
    servo6.writeMicroseconds(motor_6 = motor_6*10 < 950 ? 950 :motor_6*10 > 1200 ? 1200 : motor_6*10);
    
    //Serial.println(input_2);
    vTaskDelay(1/ portTICK_PERIOD_MS);
  }

}

void TaskCore1BLDC (void* parameter)
{
  for(;;)
  {
    double Time = micros();
    if(Time < 8000)
    {
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
    }
    else
    {
      test();
    }
    vTaskSuspend(Task4);
    vTaskDelay(1/ portTICK_PERIOD_MS);
    
  }
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&deli_package, incomingData, sizeof(deli_package));
  Serial.print(deli_package.A);
  strlength = deli_package.A.length();
  //Serial.println(strlength);
  Data = deli_package.A;
  char res = deli_package.A.charAt(strlength-strlength);
  Serial.println(res);
  switch (res)
  {
    case 'A':
    vTaskSuspend(Task3);
    Serial.println("PID OFF");
    Serial.println("State of task is");
    allstop();
    //Serial.print((int)eTaskGetState(Task3));
    break;
    case 'B':
    vTaskResume(Task3);
    Serial.println("PID ON");
    Serial.println("State of task is");
    //Serial.print((String)eTaskGetState(Task3));
    break;
    case 'C':
    Serial.println("Resume all Tasks"); 
    vTaskResume(Task1);
    vTaskResume(Task2);
    break;
    case 'D':
    Serial.println("Entering Calibration mode:");
    vTaskResume(Task4);
    break;
    case 'E':
    Serial.println("Pause all");
    vTaskSuspend(Task1);
    vTaskSuspend(Task2);
    vTaskSuspend(Task3);
    vTaskSuspend(Task4);
    allstop();
    break;
    case 'F':
    dex = deli_package.A.indexOf("F");
    height = deli_package.A.substring(1,strlength-1);
    Serial.println(height);
    setpoint_1 = height.toDouble();
    break;
    case 'P':
    dex = deli_package.A.indexOf("P");
    rev_Pid = deli_package.A.substring(1,strlength-1);
    Serial.println(rev_Pid);
    Kp_2,Kp_3 = rev_Pid.toDouble();
    break;
    case 'I':
    dex = deli_package.A.indexOf("I");
    rev_Pid = deli_package.A.substring(1,strlength-1);
    Serial.println(rev_Pid);
    Ki_2,Ki_3 = rev_Pid.toDouble(); 
    break;   
    default:
    Serial.println("Wrong Code");
    break;
  }
}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= 1100; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        servo1.writeMicroseconds(i);
        servo2.writeMicroseconds(i);
        servo3.writeMicroseconds(i);
        servo4.writeMicroseconds(i);
        servo5.writeMicroseconds(i);
        servo6.writeMicroseconds(i);
        delay(100);
    }

    Serial.println("STOP");
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
}
void allstop()
{
    servo1.writeMicroseconds(MIN_PULSE_LENGTH);
    servo2.writeMicroseconds(MIN_PULSE_LENGTH);
    servo3.writeMicroseconds(MIN_PULSE_LENGTH);
    servo4.writeMicroseconds(MIN_PULSE_LENGTH);
    servo5.writeMicroseconds(MIN_PULSE_LENGTH);
    servo6.writeMicroseconds(MIN_PULSE_LENGTH);
}
double Constraint (double low, double high, double value)
{
  value = value < low ? low :
  value > high ? high:
  value;
  return value*10;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}