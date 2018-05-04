/**************************************************************************/
/**
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
 * \file WriteReadBytes.ino
 ******************************************************************************/
/** @par Connect as follows:
-# for arduino UNO
	- Arduino UNO analog pin 4 to EEPROM pin 5 SDA
	- Arduino UNO analog pin 5 to EEPROM pin 6 SCL
	- Arduino UNO 5V to EEPROM pin 8
	- Arduino UNO GND to EEPROM pin 1,2,3,4
	.
-# for arduino Pilo
  - on P_COM4.wire
  	- PILO,       connector P_COM4  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM4  pin 5   to   EEPROM pin 6 SCL
	- PILO 3.3V,  connector P_COM4  pin 3   to   EEPROM pin 8
	- PILO GND,   connector P_COM4  pin 4   to   EEPROM pin 1,2,3,4
	.
  - on P_COM2.wire
  	- PILO,       connector P_COM2  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM2  pin 5   to   EEPROM pin 6 SCL
	- PILO 3.3V,  connector P_COM2  pin 3   to   EEPROM pin 8
	- PILO GND,   connector P_COM2  pin 4   to   EEPROM pin 1,2,3,4
	.
  - on P_COM0.wire
	- PILO,       connector P_COM0  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM0  pin 5   to   EEPROM pin 6 SCL
	- PILO 3.3V,  connector P_COM0  pin 3   to   EEPROM pin 8
	- PILO GND,   connector P_COM0  pin 4   to   EEPROM pin 1,2,3,4
	.
  - on P_COM0_BIS.wire
	- PILO,       connector P_COM0_BIS  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM0_BIS  pin 5   to   EEPROM pin 6 SCL
	- PILO 5V,    connector P_COM0_BIS  pin 2   to   EEPROM pin 8
	- PILO GND,   connector P_COM0_BIS  pin 4   to   EEPROM pin 1,2,3,4
	.
  - on P_COM1.wire
	- PILO,       connector P_COM1  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM1  pin 5   to   EEPROM pin 6 SCL
	- PILO 5V,    connector P_COM1  pin 2   to   EEPROM pin 8
	- PILO GND,   connector P_COM1  pin 4   to   EEPROM pin 1,2,3,4
	.
  - on P_COM5.wire
	- PILO,       connector P_COM5  pin 6   to   EEPROM pin 5 SDA
	- PILO,       connector P_COM5  pin 5   to   EEPROM pin 6 SCL
	- PILO 5V,    connector P_COM5  pin 2   to   EEPROM pin 8
	- PILO GND,   connector P_COM5  pin 4   to   EEPROM pin 1,2,3,4
	.
  .
.
*/
/******************************************************************************
 * Header file inclusions.
 ******************************************************************************/

#include <Wire.h>

#include <ZEeprom.h>
#include <WireUtility.h>
/******************************************************************************
 * Private macro definitions.
 ******************************************************************************/

/**************************************************************************//**
 * \def EEPROM_ADDRESS
 * \brief Address of EEPROM memory on TWI bus.
 ******************************************************************************/
#define EEPROM_ADDRESS AT24Cxx_BASE_ADDR

/******************************************************************************
 * Private variable definitions.
 ******************************************************************************/

 ZEeprom * eeprom;

/******************************************************************************
 * Public function definitions.
 ******************************************************************************/
 #ifdef ARDUINO_AVR_UNO
 #define MyWire Wire 
#define MySerial Serial 
 #elif defined(BOARD_ID_Pilo)
//#define MyWire P_COM0.wire
//#define MyWire P_COM4.wire
//#define MyWire P_COM2.wire

#define MyWire P_COM0_BIS.wire
//#define MyWire P_COM1.wire
//#define MyWire P_COM5.wire
#define MySerial P_COM3.serial2
 #elif defined(BOARD_ID_Captor)

//#define MyWire P_COM0.wire //sercom0
//#define MySerial P_COM1.serial //sercom3

#define MyWire WireA //sercom3
#define MySerial P_COM0.serial2 // sercom0


//#define MyWire WireB //sercom1
//#define MySerial P_COM0.serial2 // sercom0

#endif

/**************************************************************************//**
 * \fn void setup()
 *
 * \brief
 ******************************************************************************/
void setup()
{
    // Initialize MySerial communication.
    MySerial.begin(115200);//9600
     MySerial.println("Setup...");
     scan(MySerial,MyWire);
    while(scanNext(MySerial,MyWire)!=0);
     
    // Initiliaze EEPROM library.
    eeprom= new ZEeprom();
    eeprom->begin(MyWire,AT24Cxx_BASE_ADDR,AT24C02);

    const byte address = 0;
    const byte count = 94;

    // Declare byte arrays.
    byte inputBytes[count] = { 0 };
    byte outputBytes[count] = { 0 };

    // Fill input array with printable characters. See ASCII table for more
    // details.
    for (byte i = 0; i < count; i++)
    {    
        inputBytes[i] = i + 33;
    }

    // Write input array to EEPROM memory.
    MySerial.println("Write bytes to EEPROM memory...");
    eeprom->writeBytes(address, count, inputBytes);

    // Read array with bytes read from EEPROM memory.
    MySerial.println("Read bytes from EEPROM memory...");
    eeprom->readBytes(address, count, outputBytes);
    
    // Print read bytes.
    MySerial.println("Read bytes:");
    for (byte i = 0; i < count; i++)
    {
        MySerial.write(outputBytes[i]);
        MySerial.print(" ");
    }
    MySerial.println("");
}

/**************************************************************************//**
 * \fn void loop()
 *
 * \brief
 ******************************************************************************/
void loop()
{

}
