#ifndef mpu9250_h
#define mpu9250_h
#include<stdint.h>
#include <Arduino.h>
#include <stdlib.h>

static uint32_t crc32_tab[] =
{
    0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
    0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
    0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
    0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
    0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
    0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
    0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
    0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
    0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
    0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
    0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
    0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
    0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
    0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
    0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
    0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
    0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
    0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
    0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
    0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
    0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
    0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
    0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
    0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
    0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
    0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
    0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
    0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
    0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
    0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
    0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
    0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
    0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
    0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
    0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
    0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
    0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
    0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
    0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
    0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
    0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
    0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
    0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
    0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
    0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
    0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
    0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
    0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
    0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
    0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
    0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
    0x2d02ef8dL
};

enum MPU9250Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum MPU9250Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum MPU9250Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

typedef struct mpu9250
{
	int type = 11;
	int id = 1;
	int sequence = 2;
}t_mpu9250;

class MPU9250 
{
public:
	MPU9250();
	~MPU9250();
	void begin(void);
	void MPU9250getMres(void);
	void MPU9250getGres(void);
	void MPU9250getAres(void);
	void MPU9250readAccelData(int16_t * destination);
	void MPU9250readGyroData(int16_t * destination);
	void MPU9250readMagData(int16_t * destination);
	int16_t MPU9250readTempData(void);
	void initAK8963(float * destination);
	void initMPU9250(void);
	void accelgyrocalMPU9250(float * dest1, float * dest2);
	void magcalMPU9250(float * dest1);
	void MPU9250SelfTest(float * destination);
	void I2Cscan(void);
	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	uint8_t getDevice(uint8_t address, uint8_t subaddress);
	uint8_t getData(uint8_t address, uint8_t subaddress);
  void getMpu9250Data(float* sensordata);
  void updatebuf(void);
  void updatesensorbuf(void);
  void Createpacket(uint8_t* buf);
  uint32_t crc32(const char* s, int len);
	void end(void);
private:
	uint8_t readByte(uint8_t address, uint8_t subAddress);
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	uint8_t MPU9250Gscale = GFS_250DPS;
	uint8_t MPU9250Ascale = AFS_2G;
	uint8_t MPU9250Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t MPU9250Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	float MPU9250aRes, MPU9250gRes, MPU9250mRes;             // scale resolutions per LSB for the sensors

	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float magCalibration[3] = { 0, 0, 0 };  // Factory mag calibration and mag bias
	float MPU9250gyroBias[3] = { 0, 0, 0 }, MPU9250accelBias[3] = { 0, 0, 0 }, MPU9250magBias[3] = { 0, 0, 0 };      // Bias corrections for gyro and accelerometer
	int16_t tempCount;            // temperature raw count output
	float   temperature, altitude;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
	float SelfTest[6];            // holds results of gyro and accelerometer self test

								  // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	float pi = 3.14159f;
	float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = pi * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
												  // There is a tradeoff in the beta parameter between accuracy and response speed.
												  // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
												  // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
												  // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
												  // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
												  // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
												  // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
												  // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
	#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	#define Ki 0.0f

	uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
	float pitch, yaw, roll;
	float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
	float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
	uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	uint32_t Now = 0;                         // used to calculate integration interval
  double Timecount = 0.0f, OldTimecount = 0.0f, NowTimecount = 0.0f;
	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest MPU9250 sensor data values 
	float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
	float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };    // vector to hold quaternion
	float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method

  void profloattohex(float* databuf, uint8_t* res);
  void float_to_hex(const float& val, char hex[4]);
  long floattohex(float hex);
};


extern MPU9250 CmelodyMPU9250;
#endif
