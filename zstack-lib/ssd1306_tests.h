#ifndef __SSD1306_TEST_H__
#define __SSD1306_TEST_H__

#include "delay.h"

void ssd1306_TestBorder(void);
void ssd1306_TestFonts(void);
void ssd1306_TestFPS(void);
void ssd1306_TestAll(void);
void ssd1306_TestLine(void);
void ssd1306_TestRectangle(void);
void ssd1306_TestCircle(void);
void ssd1306_TestArc(void);
void ssd1306_TestPolyline(void);
void ssd1306_TestDrawBitmap(void);
void ssd1306_DisplayTemperature(void);
void ssd1306_DisplayHumidity(void);
void ssd1306_DisplaySensorFrame(uint16 temp, uint16 humi, bool connected, uint8 battPercent);
void ssd1306_DisplaySensor(uint16 temp, uint16 humi, bool connected, uint8 battPercent);
void ssd1306_DisplayConnector(bool connect);
void ssd1306_DisplayBattery(uint8 battPercent);
void ssd1306_DisplayFactoryResetter(void);
#endif // __SSD1306_TEST_H__
