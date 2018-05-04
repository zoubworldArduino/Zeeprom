/**************************************************************************//**
 * \brief EEPROM 24C01 /02/04/08/16/32/64/128/254/512/1024 library for Arduino - Demonstration program
 * \author Copyright (C) 2018  Pierre Valleau - www.zoubworld.com
 * \version 2.0
 * \date 201804224
 * Based on the work of  Julien Le Sech - www.idreammicro.com( Copyright (C) 2012 )
 * source : https://github.com/jlesech/Eeprom24C01_02.git 
 * Extended to 04/08/16/32/64/128/254/512/1024 Eeprom
 *
 * This file is part of the EEPROM 24C01 /02/04/08/16/32/64/128/254/512/1024 library for Arduino.
 *
 * This library is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 * 
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/
 ******************************************************************************/

/**************************************************************************//**
 * \file ZEeprom.cpp
 ******************************************************************************/

/******************************************************************************
 * Header file inclusions.
 ******************************************************************************/

#include <Arduino.h>



#include <ZEeprom.h>

/******************************************************************************
 * Private macro definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \def EEPROM__PAGE_SIZE
 * \brief Size of a page in EEPROM memory.
 * This size is given by EEPROM memory datasheet.
 ******************************************************************************/
#define EEPROM__PAGE_SIZE         PAGE_LENGTH()

/**************************************************************************//**
 * \def EEPROM__RD_BUFFER_SIZE
 * \brief Size of input TWI buffer.
 * This size is equal to BUFFER_LENGTH defined in _i2c library (32 bytes).
 ******************************************************************************/
#ifndef BUFFER_LENGTH
#define BUFFER_LENGTH SERIAL_BUFFER_SIZE
#endif

#define EEPROM__RD_BUFFER_SIZE    BUFFER_LENGTH

/**************************************************************************//**
 * \def EEPROM__WR_BUFFER_SIZE
 * \brief Size of output TWI buffer.
 * This size is equal to BUFFER_LENGTH - 1 byte reserved for address.
 ******************************************************************************/
#define EEPROM__WR_BUFFER_SIZE    (BUFFER_LENGTH - 1)


/** compute the device address, sometime the device address content the data address MSB

*/
 #define DEVICEADDRESS ((m_deviceAddress&~(memorytype>>24)) | ((memorytype>>24)& address))
/** compute the data address, sometime the device address content the data address MSB, some bit are on a 2nd byte.
 */
 #define DATAADDRESS (address&memorytype&0x00ffffff)
 
/******************************************************************************
 * Public method definitions.
 ******************************************************************************/

 
/**************************************************************************//**
 * \fn ZEeprom::ZEeprom(byte deviceAddress)
 *
 * \brief Constructor.
 *
 * \param   deviceAddress   EEPROM address on TWI bus.
 ******************************************************************************/
ZEeprom::ZEeprom
(
)
{
  SerialDebug=0;
}


void ZEeprom::setSerialDebug(Uart * mySerialDebug)
{
  SerialDebug=mySerialDebug;
}
int ZEeprom::PAGE_LENGTH()
{
	
  switch(memorytype)
    {
    case AT24C01A:
    case  AT24C02:
      return 8;
    break;
    case  AT24C04:
    case  AT24C08A:
    case  AT24C16A:
      return 16;
    break;
    case  AT24C32:
    case  AT24C64:
      return 32;
    break;
    case  AT24C128:
    case  AT24C256:
      return 64;
    break;
    case  AT24C512:
      return 128;
    break;
    case  AT24C1024:
      return 256;
    break;    
    default :
  while(1);
  }
  return -1;
}
/**************************************************************************//**
 * \fn void ZEeprom::writeByte(
 * unsigned int address,
 * byte data)
 *
 * \brief Write a byte in EEPROM memory.
 *
 * \remarks A delay of 10 ms is required after write cycle.
 *
 * \param   address Address.
 * \param   data    Byte to write.
 ******************************************************************************/

 
void
ZEeprom::writeByte
(
    unsigned int    address,
    byte    data
){
	if (SerialDebug) {SerialDebug->print("writeByte(");SerialDebug->print(address);SerialDebug->println(")");}
   
    _i2c->beginTransmission((uint8_t)DEVICEADDRESS);
	if (((memorytype>>16) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    if (((memorytype>>8) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    _i2c->write(DATAADDRESS);	
    _i2c->write(data);
    _i2c->endTransmission();
}

/**************************************************************************//**
 * \fn void ZEeprom::writeBytes(
 * unsigned int     address,
 * unsigned int     length,
 * byte*    p_data)
 * 
 * \brief Write bytes in EEPROM memory.
 *
 * \param       address Start address.
 * \param       length  Number of bytes to write.
 * \param[in]   p_data  Bytes to write.
 ******************************************************************************/
void
ZEeprom::writeBytes
(
    unsigned int    address,
    unsigned int    length,
    byte*   p_data
){
	if (SerialDebug) {SerialDebug->print("writeBytes(");SerialDebug->print(address);SerialDebug->print(",");
        SerialDebug->print(length);SerialDebug->println(")");}
   
    // Write first page if not aligned.
    byte notAlignedLength = 0;
    byte pageOffset = address % EEPROM__PAGE_SIZE;
    if (pageOffset > 0)
    {
        notAlignedLength = EEPROM__PAGE_SIZE - pageOffset;
        if (length < notAlignedLength)
        {
            notAlignedLength = length;
        }
		Serial.println("writePage1");
        writePage(address, notAlignedLength, p_data);
        length -= notAlignedLength;
    }

    if (length > 0)
    {
        address += notAlignedLength;
        p_data += notAlignedLength;

        // Write complete and aligned pages.
        byte pageCount = length / EEPROM__PAGE_SIZE;
        for (byte i = 0; i < pageCount; i++)
        {
            if (SerialDebug) {SerialDebug->println("writePage2");
			SerialDebug->print("pageCount=");SerialDebug->println(pageCount);
			SerialDebug->print("i=");SerialDebug->println(i);}
            writePage(address, EEPROM__PAGE_SIZE, p_data);
            address += EEPROM__PAGE_SIZE;
            p_data += EEPROM__PAGE_SIZE;
            length -= EEPROM__PAGE_SIZE;
        }

        if (length > 0)
        {
            // Write remaining uncomplete page.
         if (SerialDebug) SerialDebug->println("writePage3");
         writePage(address, length, p_data);
        }
    }
}

/**************************************************************************//**
 * \fn byte ZEeprom::readByte(unsigned int address)
 * 
 * \brief Read a byte in EEPROM memory.
 *
 * \param   address Address.
 *
 * \return Read byte.
 ******************************************************************************/
byte  ZEeprom::readByte
(
    unsigned int address
){
	if (SerialDebug) {SerialDebug->print("readByte(");SerialDebug->print(address);SerialDebug->println(")");}
   
    _i2c->beginTransmission((uint8_t)DEVICEADDRESS);
	if (((memorytype>>16) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    if (((memorytype>>8) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    _i2c->write(DATAADDRESS);
    _i2c->endTransmission();
    _i2c->requestFrom((uint8_t)DEVICEADDRESS, (byte)1);
    byte data = 0;
    if (_i2c->available())
    {
        data = _i2c->read();
    }
    return data;
}

/**************************************************************************//**
 * \fn void ZEeprom::readBytes(
 * unsigned int     address,
 * unsigned int     length,
 * byte*    p_data)
 *
 * \brief Read bytes in EEPROM memory.
 *
 * \param       address Start address.
 * \param       length  Number of bytes to read.
 * \patam[in]   p_data  Byte array to fill with read bytes.
 ******************************************************************************/
void
ZEeprom::readBytes
(
    unsigned int    address,
    unsigned int    length,
    byte*   p_data
){
	if (SerialDebug) {SerialDebug->print("readBytes(");SerialDebug->print(address);SerialDebug->print(",");SerialDebug->print(length);SerialDebug->println(")");}
   
    byte bufferCount = length / EEPROM__RD_BUFFER_SIZE;
    for (byte i = 0; i < bufferCount; i++)
    {
        byte offset = i * EEPROM__RD_BUFFER_SIZE;
        readBuffer(address + offset, EEPROM__RD_BUFFER_SIZE, p_data + offset);
    }

    byte remainingBytes = length % EEPROM__RD_BUFFER_SIZE;
    byte offset = length - remainingBytes;
    readBuffer(address + offset, remainingBytes, p_data + offset);
}

/******************************************************************************
 * Private method definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \fn void ZEeprom::writePage(
 * unsigned int     address,
 * unsigned int     length,
 * byte*    p_data)
 *
 * \brief Write page in EEPROM memory.
 *
 * \param       address Start address.
 * \param       length  Number of bytes (EEPROM__PAGE_SIZE bytes max).
 * \param[in]   p_data  Data.
 ******************************************************************************/
void
ZEeprom::writePage
(
    unsigned int    address,
    unsigned int    length,
    byte*   p_data
){
	if (SerialDebug) {SerialDebug->print("writePage(");SerialDebug->print(address);SerialDebug->print(",");SerialDebug->print(length);SerialDebug->println(")");}
   
//	int size=min(EEPROM__WR_BUFFER_SIZE,length);
    // Write complete buffers.
    byte bufferCount = length / EEPROM__WR_BUFFER_SIZE	;
    for (byte i = 0; i < bufferCount; i++)
    {
        byte offset = i * EEPROM__WR_BUFFER_SIZE;
        writeBuffer(address + offset, EEPROM__WR_BUFFER_SIZE, p_data + offset);
    }

    // Write remaining bytes.
    byte remainingBytes = length % EEPROM__WR_BUFFER_SIZE;
    byte offset = length - remainingBytes;
    writeBuffer(address + offset, remainingBytes, p_data + offset);
}


/**************************************************************************//**
 * \fn void ZEeprom::writeBuffer(
 * unsigned int     address,
 * unsigned int     length,
 * byte*    p_data)
 *
 * \brief Write bytes into memory.
 *
 * \param       address Start address.
 * \param       length  Number of bytes (EEPROM__WR_BUFFER_SIZE bytes max).
 * \param[in]   p_data  Data.
 ******************************************************************************/
void
ZEeprom::writeBuffer
(
    unsigned int    address,
    unsigned int    length,
    byte*   p_data
){
	if (SerialDebug) {SerialDebug->print("writeBuffer(");SerialDebug->print(address);SerialDebug->print(",");SerialDebug->print(length);SerialDebug->println(")");}
   
    _i2c->beginTransmission((uint8_t)DEVICEADDRESS);
	if (((memorytype>>16) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    if (((memorytype>>8) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    _i2c->write(DATAADDRESS);
    for (byte i = 0; i < length; i++)
    {
        _i2c->write(p_data[i]);
    }
    int res=_i2c->endTransmission();
    if (SerialDebug) SerialDebug->println(res);
    // Write cycle time (tWR). See EEPROM memory datasheet for more details.
    delay(10);
}

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/


  void ZEeprom::begin(TwoWire &i2c,uint8_t addr,unsigned int   metype)
  {
    memorytype=metype;
    
  _i2c = &i2c;

  m_deviceAddress = addr;
  
   _i2c->begin();
}
  void ZEeprom::begin(TwoWire &i2c,uint8_t addr)
  {
  begin(i2c,addr,    AT24C512);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void ZEeprom::begin(uint8_t addr)
  {
    begin(Wire, addr);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void ZEeprom::begin(void)
  {
	  begin(0x50);
  }
  
/**************************************************************************//**
 * \fn void ZEeprom::readBuffer(
 * unsigned int     address,
 * unsigned int     length,
 * byte*    p_data)
 *
 * \brief Read bytes in memory.
 *
 * \param       address Start address.
 * \param       length  Number of bytes (EEPROM__RD_BUFFER_SIZE bytes max).
 * \param[in]   p_data  Buffer to fill with read bytes.
 ******************************************************************************/
void
ZEeprom::readBuffer
(
    unsigned int    address,
    unsigned int    length,
    byte*   p_data
){
	if (SerialDebug) {SerialDebug->print("readBuffer(");SerialDebug->print(address);SerialDebug->print(",");SerialDebug->print(length);SerialDebug->println(")");}
    _i2c->beginTransmission((uint8_t)DEVICEADDRESS);
	if (((memorytype>>16) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    if (((memorytype>>8) &&0xff)!=0)
		_i2c->write(DATAADDRESS>>8);
    _i2c->write(DATAADDRESS);
    _i2c->endTransmission();
    _i2c->requestFrom((uint8_t)DEVICEADDRESS, length);
    for (byte i = 0; i < length; i++)
    {
        if (_i2c->available())
        {
            p_data[i] = _i2c->read();
        }
    }
}

