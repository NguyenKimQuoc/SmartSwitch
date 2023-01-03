#pragma once
//
//    FILE: PCF8574.h
//  AUTHOR: Rob Tillaart
//    DATE: 02-febr-2013
// VERSION: 0.3.7
// PURPOSE: Arduino library for PCF8574 - 8 channel I2C IO expander
//     URL: https://github.com/RobTillaart/PCF8574
//          http://forum.arduino.cc/index.php?topic=184800
//


#include "hal_i2c.h"


#define PCF8574_LIB_VERSION         ("0.3.7")

#ifndef PCF8574_INITIAL_VALUE
#define PCF8574_INITIAL_VALUE       0xFF
#endif

#define PCF8574_OK                  0x00
#define PCF8574_PIN_ERROR           0x81
#define PCF8574_I2C_ERROR           0x82

#define LOW                             0
#define HIGH                            1

#define PCF_ADDR                        (0x20<<1)
//bool    begin(uint8 value = PCF8574_INITIAL_VALUE);
//bool    isConnected(void);

int     lastError(void);


//uint8 _address = 0x20;

//  note: setting the address corrupt internal buffer values
//  a read8(void) / write8(void) call updates them.
//void    setAddress(const uint8 deviceAddress);
//uint8 getAddress(void);  


uint8 read8(void);
uint8 read(const uint8 pin);
uint8 value(void);


void    write8(const uint8 value);
void    write(const uint8 pin, const uint8 value);
uint8 valueOut(void);


//  added 0.1.07/08 Septillion

uint8 readButton8(const uint8 mask);
uint8 readButton(const uint8 pin);
void    setButtonMask(const uint8 mask);
uint8 getButtonMask(void);


//  rotate, shift, toggle, reverse expect all lines are output
void    toggle(const uint8 pin);
void    toggleMask(const uint8 mask);    // default 0xFF ==> invertAll(void)
void    shiftRight(const uint8 n);
void    shiftLeft(const uint8 n);
void    rotateRight(const uint8);
void    rotateLeft(const uint8);
void    reverse(void);


void    select(const uint8 pin);
void    selectN(const uint8 pin);
void    selectNone(void);
void    selectAll(void);






// -- END OF FILE --