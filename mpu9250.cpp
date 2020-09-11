#include <Wire.h>
#include <WiFi.h>
#include<stddef.h>
#include<math.h>
#include"mpu9250.h"
#include"mpu9250const.h"
#include "tmp117.h"
#define RESERVED_D    0x0000
#define RESERVED      0xFFFF
#define POWER         0xA5A5

#define HI_UINT16(a)  (((a) >> 8) & 0xFF)
#define LO_UINT16(a)  ((a) & 0xFF)

#define HI_UINT32(a)  (((a) >> 16) & 0xFFFF)
#define LO_UINT32(a)  ((a) & 0xFFFF)

  
WiFiServer server(800);

MPU9250::MPU9250()
{
}

MPU9250::~MPU9250()
{
}

void MPU9250::begin(void)
{
	Wire.begin();
	delay(4000);
	byte c = getDevice(MPU9250_ADDRESS, MPU9250_WHO_AM_I);
	Serial.println("MPU9250");
	Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
	if (c == 0x71)
	{
		MPU9250SelfTest(SelfTest);
    	Serial.println("MPU9250 is online..."); 
		MPU9250getAres();
		MPU9250getGres();
		MPU9250getMres();

		Serial.println("Calibrate MPU9250 gyro and accel");
		accelgyrocalMPU9250(MPU9250gyroBias, MPU9250accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		Serial.println("accel biases (mg)"); Serial.println(1000.*MPU9250accelBias[0]);
		Serial.println(1000.*MPU9250accelBias[1]); Serial.println(1000.*MPU9250accelBias[2]);
		Serial.println("gyro biases (dps)"); Serial.println(MPU9250gyroBias[0]);
		Serial.println(MPU9250gyroBias[1]); Serial.println(MPU9250gyroBias[2]);

		delay(100);
		initMPU9250();
		Serial.println("MPU9250 initialized for active data mode....");

		byte d = getDevice(AK8963_ADDRESS, WHO_AM_I_AK8963);
		Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

		delay(100);

		initAK8963(magCalibration);
		Serial.println("AK8963 initialized for active data mode....");

		magcalMPU9250(MPU9250magBias);
		Serial.println("AK8963 mag biases (mG)"); Serial.println(MPU9250magBias[0]); Serial.println(MPU9250magBias[1]); Serial.println(MPU9250magBias[2]);
		delay(100);

    server.begin();
    server.setNoDelay(true);
	}
	else
	{
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
		while (1); // Loop forever if communication doesn't happen
	}
}
void MPU9250::getMpu9250Data(float* sensordata)
{
//    float sensordata[10] = {0.0f};
    if (readByte(MPU9250_ADDRESS, MPU9250_INT_STATUS) & 0x01) {  // check if data ready interrupt
    MPU9250readAccelData(accelCount);  // Read the x/y/z adc values
    MPU9250getMres();
                       // Now we'll calculate the accleration value into actual g's
    sensordata[0] = (float)accelCount[0] * MPU9250aRes - MPU9250accelBias[0];  // get actual g value, this depends on scale being set
    sensordata[1] = (float)accelCount[1] * MPU9250aRes - MPU9250accelBias[1];
    sensordata[2] = (float)accelCount[2] * MPU9250aRes - MPU9250accelBias[2];

    MPU9250readGyroData(gyroCount);  // Read the x/y/z adc values

                     // Calculate the gyro value into actual degrees per second
    sensordata[3] = (float)gyroCount[0] * MPU9250gRes;  // get actual gyro value, this depends on scale being set
    sensordata[4] = (float)gyroCount[1] * MPU9250gRes;
    sensordata[5] = (float)gyroCount[2] * MPU9250gRes;

    MPU9250readMagData(magCount);  // Read the x/y/z adc values

                     // Calculate the magnetometer values in milliGauss
                     // Include factory calibration per data sheet and user environmental corrections
    sensordata[6] = (float)magCount[0] * MPU9250mRes*magCalibration[0] - MPU9250magBias[0];  // get actual magnetometer value, this depends on scale being set
    sensordata[7] = (float)magCount[1] * MPU9250mRes*magCalibration[1] - MPU9250magBias[1];
    sensordata[8] = (float)magCount[2] * MPU9250mRes*magCalibration[2] - MPU9250magBias[2];
  }
//    now = micros();
//    deltat = ((now - lastUpdate) / 1000000.0000f); // set integration time by time elapsed since last filter update
//    Timecount = Timecount + deltat;
//    lastUpdate = now;
//    sensordata[0] = Timecount;
    /*
    Serial.println(sensordata[0]);
    Serial.println(sensordata[1]);
    Serial.println(sensordata[2]);
    Serial.println(sensordata[3]);
    Serial.println(sensordata[4]);
    Serial.println(sensordata[5]);
    Serial.println(sensordata[6]);
    Serial.println(sensordata[7]);
    Serial.println(sensordata[8]);
    Serial.println(sensordata[9]);
*/
//    return sensordata;
}

void MPU9250::Createpacket(uint8_t* buf)
{
  uint16_t count_d = 0,count = 0;
  uint8_t sendhead[7] = {0};
  uint8_t sendbuf[28] = {0};
  sendhead[0] = 0x01;
  sendhead[1] = 0xF1;
  sendhead[2] = 0xF5;
  sendhead[3] = HI_UINT16(0x0234);
  sendhead[4] = LO_UINT16(0x0234);
  sendhead[5] = HI_UINT16(count);
  sendhead[6] = LO_UINT16(count);

  sendbuf[0] = HI_UINT16(RESERVED_D);
  sendbuf[1] = LO_UINT16(RESERVED_D);
  sendbuf[2] = HI_UINT16(count_d);
  sendbuf[3] = LO_UINT16(count_d);
  sendbuf[4] = HI_UINT16(RESERVED_D);
  sendbuf[5] = LO_UINT16(RESERVED_D);
  sendbuf[6] = HI_UINT16(CTmpDemo.getTemperature());
  sendbuf[7] = LO_UINT16(CTmpDemo.getTemperature());
  sendbuf[8] = HI_UINT16(RESERVED_D);
  sendbuf[9] = LO_UINT16(RESERVED_D);
  sendbuf[10] = HI_UINT16(POWER);
  sendbuf[11] = LO_UINT16(POWER);
  sendbuf[12] = HI_UINT16(RESERVED);
  sendbuf[13] = LO_UINT16(RESERVED);
  sendbuf[14] = HI_UINT16(RESERVED);
  sendbuf[15] = LO_UINT16(RESERVED);
  sendbuf[16] = HI_UINT16(RESERVED);
  sendbuf[17] = LO_UINT16(RESERVED);
  sendbuf[18] = HI_UINT16(RESERVED);
  sendbuf[19] = LO_UINT16(RESERVED);
  sendbuf[20] = HI_UINT16(RESERVED);
  sendbuf[21] = LO_UINT16(RESERVED);
  sendbuf[22] = HI_UINT16(RESERVED);
  sendbuf[23] = LO_UINT16(RESERVED);
  sendbuf[24] = HI_UINT16(RESERVED);
  sendbuf[25] = LO_UINT16(RESERVED);
  sendbuf[26] = HI_UINT16(RESERVED);
  sendbuf[27] = LO_UINT16(RESERVED);
  if(count_d == 0)
  {
      memcpy(buf,sendhead,7);
      memcpy(buf+7,sendbuf,28);
  }
  else
  {
    memcpy((buf + 7 + count_d * 28),sendbuf, 28);
  }

  count++;
  count_d++;
  if(count == 65535 || count_d == 20)
  {
    if(count == 65535)
        count = 0;
    else if(count_d == 20)
        {
          count_d = 0;
          //client.write(buf, 571);
        }
  }
  
}
void MPU9250::updatebuf(void)
{
  // put your main code here, to run repeatedly:
  WiFiClient client = server.available();
  uint16_t count_d = 0,count = 0;
  uint8_t buf[571] = {0};
  if(client){
    Serial.println("New Client.");
    while(client.connected())
    {
//      if(client.available())
//      {
//        char c = client.read();
//        Serial.write(c);
//
//        client.println(CTmpDemo.getTemperature(), 3);
//        Serial.println(CTmpDemo.getTemperature(), 3);
//      }
      uint8_t sendhead[7] = {0};
      uint8_t sendbuf[28] = {0};
      sendhead[0] = 0x01;
      sendhead[1] = 0xF1;
      sendhead[2] = 0xF5;
      sendhead[3] = HI_UINT16(0x0234);
      sendhead[4] = LO_UINT16(0x0234);
      sendhead[5] = HI_UINT16(count);
      sendhead[6] = LO_UINT16(count);
      
      sendbuf[0] = HI_UINT16(RESERVED_D);
      sendbuf[1] = LO_UINT16(RESERVED_D);
      sendbuf[2] = HI_UINT16(count_d);
      sendbuf[3] = LO_UINT16(count_d);
      sendbuf[4] = HI_UINT16(RESERVED_D);
      sendbuf[5] = LO_UINT16(RESERVED_D);
      sendbuf[6] = HI_UINT16(CTmpDemo.getTemperature());
      sendbuf[7] = LO_UINT16(CTmpDemo.getTemperature());
      sendbuf[8] = HI_UINT16(RESERVED_D);
      sendbuf[9] = LO_UINT16(RESERVED_D);
      sendbuf[10] = HI_UINT16(POWER);
      sendbuf[11] = LO_UINT16(POWER);
      sendbuf[12] = HI_UINT16(RESERVED);
      sendbuf[13] = LO_UINT16(RESERVED);
      sendbuf[14] = HI_UINT16(RESERVED);
      sendbuf[15] = LO_UINT16(RESERVED);
      sendbuf[16] = HI_UINT16(RESERVED);
      sendbuf[17] = LO_UINT16(RESERVED);
      sendbuf[18] = HI_UINT16(RESERVED);
      sendbuf[19] = LO_UINT16(RESERVED);
      sendbuf[20] = HI_UINT16(RESERVED);
      sendbuf[21] = LO_UINT16(RESERVED);
      sendbuf[22] = HI_UINT16(RESERVED);
      sendbuf[23] = LO_UINT16(RESERVED);
      sendbuf[24] = HI_UINT16(RESERVED);
      sendbuf[25] = LO_UINT16(RESERVED);
      sendbuf[26] = HI_UINT16(RESERVED);
      sendbuf[27] = LO_UINT16(RESERVED);

//      client.write(sendhead, 7);
//      client.write(sendbuf, 21);
      if(count_d == 0)
      {
          memcpy(buf,sendhead,7);
          memcpy(buf+7,sendbuf,28);
      }
      else
      {
        memcpy((buf + 7 + count_d * 28),sendbuf, 28);
      }

      count++;
      count_d++;
      if(count == 65535 || count_d == 20)
      {
        if(count == 65535)
            count = 0;
        else if(count_d == 20)
            {
              count_d = 0;
              client.write(buf, 571);
            }
      }
      delay(20);
    }
  }
}

void MPU9250::updatesensorbuf(void)
{
  // put your main code here, to run repeatedly:
  WiFiClient client = server.available();
  uint16_t count_d = 0,count = 0;
  uint8_t temp[571] = {0};
  double Ddatabuf[12] = {0};
  float databuf[12] = {0};
  uint8_t buf[1931] = {0};
  uint8_t crc[4] = {0};
  char* str = "123456789";
  if(client){
    Serial.println("New Client.");
    while(client.connected())
    {
      uint8_t sendhead[7] = {0};
      uint8_t countbuf[4] = {0};
      sendhead[0] = 0x01;
      sendhead[1] = 0xFE;
      sendhead[2] = 0xFA;
      sendhead[3] = HI_UINT16(0x07DB);
      sendhead[4] = LO_UINT16(0x07DB);
      sendhead[5] = HI_UINT16(count);
      sendhead[6] = LO_UINT16(count);

      getMpu9250Data(databuf);
      MadgwickQuaternionUpdate(-ax, databuf[1], databuf[2], databuf[3]*pi / 180.0f, -databuf[4]*pi / 180.0f, -databuf[5]*pi / 180.0f, databuf[7], -databuf[6], databuf[8]);
      a12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
      a22 = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
      a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
      a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
      a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
      pitch = -asinf(a32);
      roll = atan2f(a31, a33);
      yaw = atan2f(a12, a22);
      pitch *= 180.0f / PI;
      yaw *= 180.0f / PI;
      yaw -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      if (yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
      roll *= 180.0f / PI;
      databuf[9] = roll;
      databuf[10] = pitch;
      databuf[11] = yaw;
      
      Now = micros();
      deltat = ((Now - lastUpdate) / 1000000.0000f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
      if(count == 0)
      {
        Timecount = Timecount + deltat - deltat;
      }
      else
      {
        Timecount = Timecount + deltat;
      }
//      NowTimecount = NowTimecount + deltat;
//      Timecount += NowTimecount - OldTimecount - deltat;
//      OldTimecount = NowTimecount;
//      Serial.println(deltat);
      /*
      Serial.println(q[0], 3);
      Serial.println(q[1], 3);
      Serial.println(q[2], 3);
      Serial.println(q[3], 3);
      Serial.println(databuf[0], 3);
      Serial.println(databuf[1], 3);
      Serial.println(databuf[2], 3);
      Serial.println(databuf[3], 3);
      Serial.println(databuf[4], 3);
      Serial.println(databuf[5], 3);
      Serial.println(databuf[6], 3);
      Serial.println(databuf[7], 3);
      Serial.println(databuf[8], 3);
      Serial.println(databuf[9], 3);
      Serial.println(databuf[10], 3);
      Serial.println(databuf[11], 3);
      */
      for(int i = 0; i < 12; i++)
      {
        Ddatabuf[i] = static_cast<double>(databuf[i]);
      }
      
      if(count_d == 0)
        {
            memcpy(buf,sendhead,7);
            countbuf[0] = HI_UINT16(RESERVED_D);
            countbuf[1] = LO_UINT16(RESERVED_D);
            countbuf[2] = HI_UINT16(count_d);
            countbuf[3] = LO_UINT16(count_d); 
            memcpy(buf+7,countbuf,4);
            memcpy(buf+11,Ddatabuf,96);
        }
        else
        {
          countbuf[0] = HI_UINT16(RESERVED_D);
          countbuf[1] = LO_UINT16(RESERVED_D);
          countbuf[2] = HI_UINT16(count_d);
          countbuf[3] = LO_UINT16(count_d);
          memcpy(buf + 107 + count_d * 4,countbuf, 4); //111
          memcpy((buf + 15 + count_d * 96),Ddatabuf, 96); ///115
        }
        count++;
        count_d++;
        if(count_d == 20)
        {
          crc[0] = HI_UINT16((crc32(str,9) >> 16) & 0xFFFF);
          crc[1] = LO_UINT16((crc32(str,9) >> 16) & 0xFFFF);
          crc[2] = HI_UINT16((crc32(str,9) & 0xFFFF));
          crc[3] = LO_UINT16((crc32(str,9) & 0xFFFF));
          memcpy(buf+2007,crc,4);
          client.write(buf, 2011);
//          server.println(*buf);
//          Serial.println(*buf);
          count_d = 0;
        }

        delay(2);
//
//        if(Timecount >= 2.00)
//        {
//          Createpacket(temp);
//          client.write(temp, 571);
//        }
      }
      if(!client)
      {
        client.stop();
        Serial.println("Client is disconnect");
      }
  }
}


void MPU9250::MPU9250getMres(void)
{
	switch (MPU9250Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		MPU9250mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		MPU9250mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void MPU9250::MPU9250getGres(void)
{
	switch (MPU9250Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		MPU9250gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		MPU9250gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		MPU9250gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		MPU9250gRes = 2000.0 / 32768.0;
		break;
	}
}

void MPU9250::MPU9250getAres(void)
{
	switch (MPU9250Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		MPU9250aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		MPU9250aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		MPU9250aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		MPU9250aRes = 16.0 / 32768.0;
		break;
	}
}

void MPU9250::MPU9250readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::MPU9250readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void MPU9250::MPU9250readMagData(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}
}

int16_t MPU9250::MPU9250readTempData(void)
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, MPU9250_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}

void MPU9250::initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, MPU9250Mscale << 4 | MPU9250Mmode); // Set magnetometer data resolution and sample ODR
	delay(10);
}

void MPU9250::initMPU9250(void)
{
	// wake up device
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	delay(100); // Wait for all registers to reset 

				// get stable time source
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
														   // determined inset in CONFIG above

														   // Set gyroscope full scale range
														   // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG);
	//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0] 
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c | MPU9250Gscale << 3); // Set full scale range for the gyro
																			 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

																			 // Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG);
	//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c | MPU9250Ascale << 3); // Set full scale range for the accelerometer 

																			  // Set accelerometer sample rate configuration
																			  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
																			  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2);
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

																 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
																 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

																 // Configure Interrupts and Bypass Enable
																 // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
																 // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
																 // can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}

void MPU9250::accelgyrocalMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

										 // Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO  
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

			   // At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];

	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

									 // Apparently this is not working for the acceleration biases in the MPU-9250
									 // Are we handling the temperature correction bit properly?
									 // Push accelerometer biases to hardware registers
									 /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
									 writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
									 */
									 // Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void MPU9250::magcalMPU9250(float * dest1)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(40);

	sample_count = 64;
	for (ii = 0; ii < sample_count; ii++) {
		MPU9250readMagData(mag_temp);  // Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
	}
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	dest1[0] = (float)mag_bias[0] * MPU9250mRes*magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (float)mag_bias[1] * MPU9250mRes*magCalibration[1];
	dest1[2] = (float)mag_bias[2] * MPU9250mRes*magCalibration[2];

	Serial.println("Mag Calibration done!");
}

void MPU9250::MPU9250SelfTest(float * destination)
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1 << FS);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for (uint16_t ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);
	delay(25);  // Delay a while to let the device stabilize

				// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

																		// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

																					  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
																					  // To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
		destination[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
	}
}

void MPU9250::I2Cscan(void)
{
	// scan for i2c devices
	byte error, address;
	uint16_t nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknown error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
}

void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

uint8_t MPU9250::getDevice(uint8_t address, uint8_t subaddress)
{
	return readByte(address, subaddress);
}

uint8_t MPU9250::getData(uint8_t address, uint8_t subaddress)
{
	return readByte(address, subaddress);
}


void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data = 0; // `data` will store the register data   
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);                  // Put slave register address in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
										//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
										//  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t)1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
											 //  Serial.println(data);
	return data;                             // Return data read from slave register
}

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
								  //  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
	Wire.requestFrom(address, (size_t)count);  // Read bytes from slave register address 
	while (Wire.available()) {
		dest[i++] = Wire.read();
	}         // Put read results in the Rx buffer
}
void MPU9250::profloattohex(float* databuf, uint8_t* res)
{
  for(int i = 0; i < sizeof(databuf); i++)
  {
      res[i*4 + 0] = HI_UINT16(HI_UINT32(floattohex(databuf[i])));
      res[i*4 + 1] = LO_UINT16(HI_UINT32(floattohex(databuf[i])));
      res[i*4 + 2] = HI_UINT16(LO_UINT32(floattohex(databuf[i])));
      res[i*4 + 3] = LO_UINT16(LO_UINT32(floattohex(databuf[i])));
  }
}
void MPU9250::float_to_hex(const float& val, char hex[4])
{
  const char *p  = (char*)(&val);

  hex[0] = *((char*)p);
  hex[1] = *((char*)p + 1);
  hex[2] = *((char*)p + 2);
  hex[3] = *((char*)p + 3);
}

long MPU9250::floattohex(float hex)
{
  return *(long*)&hex;
}

uint32_t MPU9250::crc32(const char* s, int len)
{
    int i;
    uint32_t crc32val = 0;
    crc32val ^= 0xFFFFFFFF;

    for (i = 0;  i < len;  i++) {
        crc32val = crc32_tab[(crc32val ^ s[i]) & 0xFF] ^ ((crc32val >> 8) & 0x00FFFFFF);
    }

    return labs(crc32val ^ 0xFFFFFFFF);
}
MPU9250 CmelodyMPU9250;
