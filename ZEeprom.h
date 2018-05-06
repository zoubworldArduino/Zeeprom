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
 * \headerfile ZEeprom.h
 ******************************************************************************/

#ifndef ZEeprom_h
#define ZEeprom_h

/******************************************************************************
 * Header file inclusion.
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>

//0xddaaaaaa
//if a=1 so bit is used for address of data
//if d=1 so bit is used for address of data


/** the type of EEProm content the coding value for address system : 0xddaaaaaa, 
	- aa refer to data address sent after the device address, 
	- dd refers to MSB of data address apply on the device address.(it's a tip to save a 2nd(or 3rd) byte of data address)
*/
#define   AT24C01A     ((uint32_t)0x0000007F)
#define   AT24C02      ((uint32_t)0x000000FF)
#define   AT24C04      ((uint32_t)0x010000FF)
#define   AT24C08A     ((uint32_t)0x030000FF)
#define   AT24C16A     ((uint32_t)0x070000FF)


#define   AT24C32      ((uint32_t)0x00000fff)
#define   AT24C64      ((uint32_t)0x00001fff)
#define   AT24C128     ((uint32_t)0x00003fff)
#define   AT24C256     ((uint32_t)0x00007fff)
#define   AT24C512     ((uint32_t)0x0000ffff)
#define   AT24C1024    ((uint32_t)0x0100ffff)

#define   AT24Cxx_BASE_ADDR 0x50

/**************************************************************************//**
 * \class ZEeprom
 *
 * \brief EEPROM 24C01 / 24C02 memory driver.
 *
 * This driver is designed for 24C01 and 24C02 EEPROM memories.
 ******************************************************************************/
class ZEeprom
{
    public:
    
        /******************************************************************//**
         * \fn ZEeprom(byte deviceAddress)
         *
         * \brief Constructor.
         *
         * \param   deviceAddress   EEPROM address on TWI bus.
         **********************************************************************/
        ZEeprom
        (
        );

		 void begin(TwoWire &MyWire,uint8_t addr,unsigned int   memorytype);
		 void begin(TwoWire &MyWire,uint8_t addr);
		 void begin(uint8_t addr);
	     void begin(void);

        /******************************************************************//**
         * \fn void initialize()
         *
         * \brief Initialize library abnd TWI bus.
         * 
         * If several devices are connected to TWI bus, this method mustn't be
         * called. TWI bus must be initialized out of this library using
         * Wire.begin() method.
         **********************************************************************/        
        void
        initialize();

        /******************************************************************//**
         * \fn void writeByte(
         * unsigned int address,
         * byte data)
         *
         * \brief Write a byte in EEPROM memory.
         *
         * \remarks A delay of 10 ms is required after write cycle.
         *
         * \param   address Address.
         * \param   data    Byte to write.
         **********************************************************************/
        void
        writeByte
        (
            unsigned int    address,
            byte    data
        );
        
        /******************************************************************//**
         * \fn void writeBytes(
         * unsigned int     address,
         * unsigned int     length,
         * byte*    p_data)
         * 
         * \brief Write bytes in EEPROM memory.
         *
         * \param       address Start address.
         * \param       length  Number of bytes to write.
         * \param[in]   p_data  Bytes to write.
         **********************************************************************/
        void
        writeBytes
        (
            unsigned int    address,
            unsigned int    length,
            byte*   p_data
        );
        
        /******************************************************************//**
         * \fn byte readByte(unsigned int address)
         * 
         * \brief Read a byte in EEPROM memory.
         *
         * \param   address Address.
         *
         * \return Read byte.
         **********************************************************************/
        byte
        readByte
        (
            unsigned int    address
        );

        /******************************************************************//**
         * \fn void readBytes(
         * unsigned int     address,
         * unsigned int     length,
         * byte*    p_data)
         *
         * \brief Read bytes in EEPROM memory.
         *
         * \param       address Start address.
         * \param       length  Number of bytes to read.
         * \patam[in]   p_data  Byte array to fill with read bytes.
         **********************************************************************/
        void
        readBytes
        (
            unsigned int    address,
            unsigned int    length,
            byte*   p_buffer
        );
        void setSerialDebug(HardwareSerial * mySerialDebug);
    private:
      int PAGE_LENGTH();
      uint32_t   memorytype;
        byte m_deviceAddress;
         TwoWire *_i2c; 
          HardwareSerial * SerialDebug;
        /******************************************************************//**
         * \fn void writePage(
         * unsigned int     address,
         * unsigned int     length,
         * byte*    p_data)
         *
         * \brief Write page in EEPROM memory.
         *
         * \param       address Start address.
         * \param       length  Number of bytes (64 bytes max).
         * \param[in]   p_data  Data.
         **********************************************************************/
        void
        writePage
        (
            unsigned int    address,
            unsigned int    length,
            byte*   p_data
        );

        /******************************************************************//**
         * \fn void writeBuffer(
         * unsigned int     address,
         * unsigned int     length,
         * byte*    p_data)
         *
         * \brief Write bytes into memory.
         *
         * \param       address Start address.
         * \param       length  Number of bytes (30 bytes max).
         * \param[in]   p_date  Data.
         **********************************************************************/
        void
        writeBuffer
        (
            unsigned int    address,
            unsigned int    length,
            byte*   p_data
       );

        /******************************************************************//**
         * \fn void readBuffer(
         * unsigned int     address,
         * unsigned int     length,
         * byte*    p_data)
         *
         * \brief Read bytes in memory.
         *
         * \param       address Start address.
         * \param       length  Number of bytes to read (32 bytes max).
         * \param[in]   p_data  Buffer to fill with read bytes.
         **********************************************************************/
        void
        readBuffer
        (
            unsigned int    address,
            unsigned int    length,
            byte*   p_data
        );
};

#endif // ZEeprom_h

