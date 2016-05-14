/* Ioton Boards Library
 * Copyright (c) 2016 Ioton Technology
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
#include "USBTon.h"

#define ON	1
#define OFF	0
#define BATTERY_SCALE	1.4681f // Voltage divider: [(R14 + R15) / R15]

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


Serial bluetooth(PA_9, PA_10);
Serial wifi(PA_2, PA_3);
USBTon usb;
DigitalOut PWD(PB_12);
DigitalOut Mode(PB_13);
DigitalOut Reset(PC_15);

DigitalIn USER(SW_USER);
PwmOut ledRED(LED_RED);
PwmOut ledGREEN(LED_GREEN);
PwmOut ledBLUE(LED_BLUE);
AnalogIn battery(PB_1);


class Ioton
{
public:
	Ioton()
	{
		setLED(NONE);

		bluetooth.baud(115200);
		wifi.baud(115200);

		PWD.write(0);
		Mode.write(1);
		Reset.write(0);

        if (this->USERisPressed())
        {
            *((unsigned long *) 0x2001FFFC) = 0xB00710AD;
            NVIC_SystemReset();
        }

		wait_ms(1);
		initUSB();
	}

	void enableWifi(void)
	{
		PWD.write(1);
		Reset.write(1);
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
