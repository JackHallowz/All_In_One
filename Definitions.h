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
TaskHandle_t Task1, Task2, Task3, Task4;

//MS5611 Vars
double ref_pres, altitude_r, filteredval;

// MAC Address
uint8_t broadcastAddress[] = { 0xB0, 0xB2, 0x1C, 0xA8, 0x77, 0xB0 };

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
String Data, height, rev_Pid;
int strlength, dex;

typedef struct PID_Val
{
  double Kp;
  double Ki;
  double Kd;
}PID_Val;
PID_Val Pid_1[4] =
{
  {80,20,1},
  {65,1,8}, //Kd = 10(tested). Increase Kp increase react speed of motor. (7-8 degrees)
  {60,1,6},
  {60,1,6},
};





