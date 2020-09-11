#ifndef __TMP117_H__
#define __TMP117_h__
#include <Arduino.h>

#define TMP117_ADDRESS    0x48
#define TEMP_RESULT     0x00
#define CONFIGURATION   0x01
#define THIGH_LIMIT     0x02
#define TLOW_LIMIT      0x03
#define EEPROM_UL     0x04
#define EEPROM1       0x05
#define EEPROM2       0x06
#define TEMP_OFFSET     0x07
#define EEPROM3       0x08
#define DEVICE_ID     0x0F

class TMP117{
	public:
	TMP117(void);
	~TMP117(void);
	
	void begin(void);
	uint16_t getTemperature(void);
	void getTMPID(void);
	
	private:
  //TMP117 Offset Reg
  const int offsetH = 0x0C;
  const int offsetL = 0x80;
  
  //set temp threshold
  const uint8_t highlimH = 0x32; // high byte of high limit
  const uint8_t highlimL = 0x00; // low byte of high lim- high 27 C
  const uint8_t lowlimH = 0x0C; // high byte of low lim
  const uint8_t lowlimL = 0x80; // low byte of low limit-low 24 C
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

};

extern TMP117 CTmpDemo;
#endif	//__TMP117_h__
