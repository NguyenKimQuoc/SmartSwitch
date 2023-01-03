//
//    FILE: PCF8574.cpp
//  AUTHOR: Rob Tillaart
//    DATE: 02-febr-2013
// VERSION: 0.3.7
// PURPOSE: Arduino library for PCF8574 - 8 channel I2C IO expander
//     URL: https://github.com/RobTillaart/PCF8574
//          http://forum.arduino.cc/index.php?topic=184800
//
// HISTORY: See CHANGELOG.md
//

#include "PCF8574.h"
static uint8 _dataIn = 0;
static uint8 _dataOut = 0xFF;
static uint8 _buttonMask = 0xFF;
static int     _error = PCF8574_OK;

//bool begin(uint8 value)
//{
////  if (! isConnected()) return false;
////  write8(value);
////  return true;
//}
//
//
//bool isConnected()
//{
//  _wire->beginTransmission(PCF_ADDR);
//  return ( _wire->endTransmission() == 0);
//}


//void setAddress(const uint8 deviceAddress)
//{
//  _address = deviceAddress;
////  return isConnected();
//}


//uint8 getAddress()
//{
//  return _address;
//}


//  removed _wire->beginTransmission(PCF_ADDR);
//  with    @100 KHz -> 265 micros()
//  without @100 KHz -> 132 micros()
//  without @400 KHz -> 52 micros()
//  TODO    @800 KHz -> ??
uint8 read8()
{
//  if (_wire->requestFrom(PCF_ADDR, (uint8)1) != 1)
//  {
//    _error = PCF8574_I2C_ERROR;
//    return _dataIn;  //  last value
//  }
  HalI2CReceive(PCF_ADDR|1, &_dataIn, 1);
//  _dataIn = _wire->read();
  return _dataIn;
}


void write8(const uint8 value)
{
  _dataOut = value;
  HalI2CSend(PCF_ADDR, &_dataOut, 1);
}


uint8 read(const uint8 pin)
{
  if (pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return 0;
  }
  read8();
  return (_dataIn & (1 << pin)) > 0;
}


void write(const uint8 pin, const uint8 value)
{
  if (pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return;
  }
  if (value == LOW)
  {
    _dataOut &= ~(1 << pin);
  }
  else
  {
    _dataOut |= (1 << pin);
  }
  write8(_dataOut);
}


void toggle(const uint8 pin)
{
  if (pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return;
  }
  toggleMask(1 << pin);
}


void toggleMask(const uint8 mask)
{
  _dataOut ^= mask;
  write8(_dataOut);
}


void shiftRight(const uint8 n)
{
  if ((n == 0) || (_dataOut == 0)) return;
  if (n > 7)         _dataOut = 0;     //  shift 8++ clears all, valid...
  if (_dataOut != 0) _dataOut >>= n;   //  only shift if there are bits set
  write8(_dataOut);
}


void shiftLeft(const uint8 n)
{
  if ((n == 0) || (_dataOut == 0)) return;
  if (n > 7)         _dataOut = 0;    //  shift 8++ clears all, valid...
  if (_dataOut != 0) _dataOut <<= n;  //  only shift if there are bits set
  write8(_dataOut);
}


int lastError()
{
  int e = _error;
  _error = PCF8574_OK;  //  reset error after read, is this wise?
  return e;
}


void rotateRight(const uint8 n)
{
  uint8 r = n & 7;
  if (r == 0) return;
  _dataOut = (_dataOut >> r) | (_dataOut << (8 - r));
  write8(_dataOut);
}


void rotateLeft(const uint8 n)
{
  rotateRight(8 - (n & 7));
}


void reverse()  //  quite fast: 14 shifts, 3 or, 3 assignment.
{
  uint8 x = _dataOut;
  x = (((x & 0xAA) >> 1) | ((x & 0x55) << 1));
  x = (((x & 0xCC) >> 2) | ((x & 0x33) << 2));
  x =          ((x >> 4) | (x << 4));
  write8(x);
}


//  added 0.1.07/08 Septillion
uint8 readButton8(const uint8 mask)
{
  uint8 temp = _dataOut;
  write8(mask | _dataOut);  //  read only selected lines
  read8();
  write8(temp);             //  restore
  return _dataIn;
}


//  added 0.1.07 Septillion
uint8 readButton(const uint8 pin)
{
  if (pin > 7)
  {
    _error = PCF8574_PIN_ERROR;
    return 0;
  }

  uint8 temp = _dataOut;
  write(pin, HIGH);
  uint8 value = read(pin);
  write8(temp);
  return value;
}


void select(const uint8 pin)
{
  uint8 n = 0x00;
  if (pin < 8) n = 1 << pin;
  write8(n);
};


void selectN(const uint8 pin) 
{
  uint8 n = 0xFF;
  if (pin < 8) n = (2 << pin) - 1;
  write8(n);
};


uint8 value(void) { 
  return _dataIn; 
}

uint8 valueOut(void) { 
  return _dataOut; 
}


void    setButtonMask(const uint8 mask) { 
  _buttonMask = mask; 
}

uint8 getButtonMask(void) { 
  return _buttonMask; 
}

void    selectNone(void) { write8(0x00); }
void    selectAll(void)  { write8(0xFF); }
// -- END OF FILE --