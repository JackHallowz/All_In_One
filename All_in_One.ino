#define RESTRICT_PITCH 
#define _eTaskGetState
#define MIN_PULSE_LENGTH 900
#define MAX_PULSE_LENGTH 1900
#define IDLE_SPEED 950 //1000 is enough
#define TOP_SPEED 1200
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
#include "Definitions.h"
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

//Declare PID Lib

double input_1,out_Thrust,input_2,out_Roll,input_3,out_Pitch,input_4,out_Yaw;
double setpoint_1,setpoint_2,setpoint_3,setpoint_4;
PID pid_Thrust(&input_1, &out_Thrust, &setpoint_1, Pid_1[0].Kp, Pid_1[0].Ki, Pid_1[0].Kd, DIRECT);
PID pid_Roll(&input_2, &out_Roll, &setpoint_2, Pid_1[1].Kp, Pid_1[1].Ki, Pid_1[1].Kd, DIRECT);
PID pid_Pitch(&input_3, &out_Pitch, &setpoint_3, Pid_1[2].Kp, Pid_1[2].Ki, Pid_1[2].Kd, DIRECT);
PID pid_Yaw(&input_4, &out_Yaw, &setpoint_4, Pid_1[3].Kp, Pid_1[3].Ki, Pid_1[3].Kd, DIRECT);

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
  pid_Thrust.SetOutputLimits(-1000, 1000);
  pid_Roll.SetMode(AUTOMATIC);
  pid_Roll.SetOutputLimits(-1000, 1000);
  pid_Pitch.SetMode(AUTOMATIC);
  pid_Pitch.SetOutputLimits(-1000, 1000);
  pid_Yaw.SetMode(AUTOMATIC);
  pid_Yaw.SetOutputLimits(-1000, 1000);
  //
  servo1.attach(servo1Pin,MIN_PULSE_LENGTH,1200);
  servo2.attach(servo2Pin,MIN_PULSE_LENGTH,1200);
  servo3.attach(servo3Pin,MIN_PULSE_LENGTH,1200);
  servo4.attach(servo4Pin,MIN_PULSE_LENGTH,1200);
  servo5.attach(servo5Pin,MIN_PULSE_LENGTH,1200);
  servo6.attach(servo6Pin,MIN_PULSE_LENGTH,1200);

  servo1.writeMicroseconds(MIN_PULSE_LENGTH);
  servo2.writeMicroseconds(MIN_PULSE_LENGTH);
  servo3.writeMicroseconds(MIN_PULSE_LENGTH);
  servo4.writeMicroseconds(MIN_PULSE_LENGTH);
  servo5.writeMicroseconds(MIN_PULSE_LENGTH);
  servo6.writeMicroseconds(MIN_PULSE_LENGTH);
 
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
 allstop();
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
    if(abs(kalAngleX) > 20 || abs(kalAngleY) > 20)
    {
      vTaskSuspend(Task3);
      allstop();
    }
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
        myData.f = motor_1;
        myData.g = motor_2;
        myData.h = motor_3;
        myData.i = motor_4;
        myData.l = motor_5;
        myData.m = motor_6;
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
    input_2 = kalAngleY; //this is 100% correct
    input_3 = kalAngleX;
    //input_4 = headingdeg;
    //pid_Thrust.Compute();
    pid_Roll.Compute();
    pid_Pitch.Compute();
    //pid_Yaw.Compute();
    // motor_1 = out_Thrust + out_Roll + out_Pitch; //front left
    // motor_2 = out_Thrust + out_Roll ; //rear left
    // motor_3 = out_Thrust + out_Roll - out_Pitch; //back left
    // motor_4 = out_Thrust - out_Roll - out_Pitch; // back right
    // motor_5 = out_Thrust - out_Roll ; //rear righ
    // motor_6 = out_Thrust - out_Roll + out_Pitch; //front right
    motor_2 = out_Thrust + out_Roll + out_Pitch; //front left
    motor_3 = out_Thrust + out_Roll - out_Pitch; //rear left
    // motor_4 = out_Thrust + out_Roll - out_Pitch; //back left
    // motor_5 = out_Thrust - out_Roll - out_Pitch; // back right
    motor_6 = out_Thrust - out_Roll - out_Pitch; //rear right
    motor_1 = out_Thrust - out_Roll + out_Pitch; //front right

    servo1.writeMicroseconds(motor_1 = motor_1 < IDLE_SPEED ? IDLE_SPEED :motor_1 > TOP_SPEED ? TOP_SPEED : motor_1);
    servo2.writeMicroseconds(motor_2 = motor_2 < IDLE_SPEED ? IDLE_SPEED :motor_2 > TOP_SPEED ? TOP_SPEED : motor_2);
    servo3.writeMicroseconds(motor_3 = motor_3 < 950 ? 950 :motor_3 > TOP_SPEED ? TOP_SPEED : motor_3);
    servo4.writeMicroseconds(motor_4 = motor_4 < IDLE_SPEED ? IDLE_SPEED :motor_4 > TOP_SPEED ? TOP_SPEED : motor_4);
    servo5.writeMicroseconds(motor_5 = motor_5 < IDLE_SPEED ? IDLE_SPEED :motor_5 > TOP_SPEED ? TOP_SPEED : motor_5);
    servo6.writeMicroseconds(motor_6 = motor_6 < 950 ? 950 :motor_6 > TOP_SPEED ? TOP_SPEED : motor_6);
    
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
  char code = deli_package.A.charAt(1);
  Serial.println(code);
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
    allstop();
    vTaskSuspend(Task1);
    vTaskSuspend(Task2);
    vTaskSuspend(Task3);
    vTaskSuspend(Task4);
    break;
    case 'F':
    dex = deli_package.A.indexOf("F");
    height = deli_package.A.substring(1,strlength-1);
    Serial.println(height);
    setpoint_1 = height.toDouble();
    break;
    case 'P':
    changevalPD(code, res);
    break;
    case 'K':
    changevalPD(code, res);
    break;
    case 'I':
    changevalPD(code, res);
    break;        
    default:
    Serial.println("Wrong Code");
    break;
  }
}
void changevalPD(char code, char cd)
{
  if (code == '2')
  {
    rev_Pid = deli_package.A.substring(2,strlength);  
    if(cd == 'P')
    {
      Pid_1[1].Kp = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKp());
    }
    else if (cd =='K')
    {
      Pid_1[1].Kd = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKd());
    }
    else if (cd =='I')
    {
      Pid_1[1].Ki = rev_Pid.toDouble();
      pid_Roll.SetTunings(Pid_1[1].Kp,Pid_1[1].Ki,Pid_1[1].Kd);
      Serial.println(pid_Roll.GetKi());
    }
  }
  else if (code == '3')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    if(cd == 'P')
    {
      Pid_1[2].Kp = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Pitch.GetKp());
    }
    else if (cd =='K')
    {
      Pid_1[2].Kd = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Pitch.GetKd());
    }
    else if (cd =='I')
    {
      Pid_1[2].Ki = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[2].Kp,Pid_1[2].Ki,Pid_1[2].Kd);
      Serial.println(pid_Roll.GetKi());
    }   
  }
  else if (code == '1')
  {
    rev_Pid = deli_package.A.substring(2,strlength);
    switch(cd)
    {
      case 'P':
      Pid_1[0].Kp = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Pitch.GetKp());
      break;
      case 'I':
      Pid_1[0].Ki = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Pitch.GetKi());
      break;
      case 'K':
      Pid_1[0].Kd = rev_Pid.toDouble();
      pid_Pitch.SetTunings(Pid_1[0].Kp,Pid_1[0].Ki,Pid_1[0].Kd);
      Serial.println(pid_Pitch.GetKd());
      break;
      default:
      break;
    }
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}