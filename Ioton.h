/* Ioton Boards Library
* Copyright (c) 2016-2017 Ioton Technology
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef IOTON_H
#define IOTON_H

#include "mbed.h"
#include "BMX055.h"
#include "ESP8266.h"
#include "USBTon.h"

#define ON  1
#define OFF 0
#define BATTERY_SCALE 1.4681f /* Voltage divider: [(R18 + R19) / R19] */
#define BATTERY_MIN   3.6f
#define BATTERY_MAX   4.2f

typedef enum
{
  RED,
  GREEN,
  BLUE,
  YELLOW,
  CYAN,
  MAGENTA,
  WHITE,
  NONE
} LEDType_t;


Serial bluetooth(BT_TX, BT_RX);
ESP8266 wifi(WIFI_TX, WIFI_RX);
DigitalOut BT_RST(PC_15);
DigitalIn VBUS(PA_9);
DigitalOut WIFI_PWD(PB_12);
DigitalOut WIFI_MODE(PB_13);
DigitalOut WIFI_RST(PC_14);

DigitalIn USER(SW_USER);
PwmOut ledRED(LED_RED);
PwmOut ledGREEN(LED_GREEN);
PwmOut ledBLUE(LED_BLUE);
AnalogIn battery(BAT_METER);

BMX055 imu;
USBTon usb;


class Ioton
{
public:
  Ioton()
  {
    setLED(NONE);
    bluetooth.baud(9600);
    WIFI_PWD.write(0);
    WIFI_MODE.write(1);
    WIFI_RST.write(0);
    BT_RST.write(0);
    wait_ms(1);

    if (VBUS) initUSB();
  }

  void batteryStatus(bool loop = true)
  {
    float batPercertage = 0;

    /* loop = true: forever  |  loop = false: only once */
    while (loop)
    {
      batPercertage = ((getBattery() - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN));

      /* GREEN: fully charged; RED: fully discharged;
        ORANGE/YELLOW: intermediate load */
      setLED(ledRED, 1.0f - batPercertage);
      setLED(ledGREEN, batPercertage);
      wait(0.5);

      /* BLINK: if it is critically or fully charged */
      if (batPercertage > 1.0f || batPercertage < 0.0f)
      {
        setLED(NONE);
        wait(0.5);
      }
    }
  }

  void enableBluetooth(void)
  {
    BT_RST.write(1);
  }

  void enableIMU(uint8_t mAscale = AFS_2G, uint8_t ACCBW  = ABW_125Hz,
    uint8_t mGscale = GFS_125DPS, uint8_t GODRBW = G_200Hz23Hz,
    uint8_t Mmode  = Regular, uint8_t MODR   = MODR_30Hz)
  {
    imu.init(mAscale, ACCBW, mGscale, GODRBW, Mmode, MODR);
  }

  void enableWifi(void)
  {
    WIFI_PWD.write(1);
    WIFI_RST.write(1);
  }

  int USERisPressed(void)
  {
    return USER;
  }

  float getBattery(void)
  {
    float vBat = battery.read();

    return (vBat * 3.3f * BATTERY_SCALE);
  }

  void setLED(PwmOut led, float intensity)
  {
    if (intensity > 1.0f)
    {
      intensity = 1.0f;
    }
    else if (intensity < 0.0f)
    {
      intensity = 0.0f;
    }

    led = 1.0f - intensity;
  }

  void setLED(LEDType_t color)
  {
    setLED(ledRED, OFF);
    setLED(ledGREEN, OFF);
    setLED(ledBLUE, OFF);

    switch(color)
    {
      case RED:
      setLED(ledRED, ON);
      break;

      case GREEN:
      setLED(ledGREEN, ON);
      break;

      case BLUE:
      setLED(ledBLUE, ON);
      break;

      case YELLOW:
      setLED(ledRED, ON);
      setLED(ledGREEN, ON);
      break;

      case CYAN:
      setLED(ledGREEN, ON);
      setLED(ledBLUE, ON);
      break;

      case MAGENTA:
      setLED(ledRED, ON);
      setLED(ledBLUE, ON);
      break;

      case WHITE:
      setLED(ledRED, ON);
      setLED(ledGREEN, ON);
      setLED(ledBLUE, ON);
      break;

      case NONE:
      break;

      default:
      break;
    }
  }

  // code: HTML Color Code (without #) - Examples: "00ff00", "70befc"
  void setLED(const char * code)
  {
    int hexValue = strtol(code, NULL, 16);

    float r = ((hexValue >> 16) & 0xFF) / 255.0;  // Extract the RR byte
    float g = ((hexValue >> 8) & 0xFF) / 255.0;   // Extract the GG byte
    float b = ((hexValue) & 0xFF) / 255.0;        // Extract the BB byte

    setLED(ledRED, r);
    setLED(ledGREEN, g);
    setLED(ledBLUE, b);
  }

  void toggleLED(LEDType_t color)
  {
    switch(color)
    {
      case RED:
      ledRED = !ledRED;
      break;

      case GREEN:
      ledGREEN = !ledGREEN;
      break;

      case BLUE:
      ledBLUE = !ledBLUE;
      break;

      default:
      break;
    }
  }
  };

  Ioton ton;

  #endif // IOTON_H
