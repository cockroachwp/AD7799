#ifndef __SMBUS_OP_H
#define __SMBUS_OP_H
unsigned char PEC_cal(unsigned char pec[]);
unsigned long int MEM_READ(unsigned char slave_addR,unsigned char cmdR,u8 id);
void EEPROM_WRITE(unsigned char slave_addW,unsigned char cmdW,unsigned char DataL,unsigned char DataH,u8 id);

#endif