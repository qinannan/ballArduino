#include <Wire.h>
#include<stddef.h>
#include<math.h>
#include "tmp117.h"

TMP117::TMP117()
{
}

TMP117::~TMP117()
{
	
}

void TMP117::begin(void)
{
  writeByte(TMP117_ADDRESS,THIGH_LIMIT,highlimH);
  writeByte(TMP117_ADDRESS,THIGH_LIMIT,highlimL);
  writeByte(TMP117_ADDRESS,TLOW_LIMIT,lowlimH);
  writeByte(TMP117_ADDRESS,TLOW_LIMIT,lowlimL);
  writeByte(TMP117_ADDRESS,CONFIGURATION,0x02);
  writeByte(TMP117_ADDRESS,CONFIGURATION,0x20);
  writeByte(TMP117_ADDRESS,TEMP_OFFSET,offsetH);
  writeByte(TMP117_ADDRESS,TEMP_OFFSET,offsetL);
  writeByte(TMP117_ADDRESS,TEMP_OFFSET,0x00);
  writeByte(TMP117_ADDRESS,TEMP_OFFSET,0x00);

  Serial.print("TMP117 ... ...  ");
  uint16_t temp_ID;
  uint8_t tmp[2] = {0};
  readBytes(TMP117_ADDRESS,DEVICE_ID,2,&tmp[0]);
  temp_ID = ((uint16_t)tmp[0] << 8) | tmp[0];
  temp_ID = temp_ID | (uint16_t)tmp[1];
  Serial.print("I AM "); Serial.print(temp_ID, HEX); Serial.print(" I should be "); Serial.println(0x0117, HEX);
}

uint16_t TMP117::getTemperature(void)
{
  uint16_t temp = 0;
  uint8_t tmp[2] = {0};
  readBytes(TMP117_ADDRESS,TEMP_RESULT,2,&tmp[0]);
  temp = ((uint16_t)tmp[0] << 8) | tmp[0];
  temp = temp | (uint16_t)tmp[1];
//  Serial.print("   Temp is  "); Serial.println((temp * 0.0078125));
//  Serial.println(temp, DEC);
//  return temp*0.0078125;
  return temp;
}

void TMP117::getTMPID(void)
{
  Serial.print("TMP117 ... ...  ");
  uint16_t temp_ID;
  uint8_t tmp[2] = {0};
  readBytes(TMP117_ADDRESS,DEVICE_ID,2,&tmp[0]);
  temp_ID = ((uint16_t)tmp[0] << 8) | tmp[0];
  temp_ID = temp_ID | (uint16_t)tmp[1];
  Serial.print("I AM "); Serial.print(temp_ID, HEX); Serial.print(" I should be "); Serial.println(0x0117, HEX);
//  return temp_ID;
}

void TMP117::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
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

void TMP117::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

TMP117 CTmpDemo;
