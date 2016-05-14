// Includes --------------------------------------------------------------------
#include "Ioton.h"


// Private variables -----------------------------------------------------------
BMX055 imu;
Ticker flipper_1;


// Flipper 0.1 s - 10 Hz -------------------------------------------------------
void flip_1(void)
{
	imu.runAHRS(0.1);
}


// *****************************************************************************
// MAIN PROGRAM ****************************************************************
int main(void)
{
	// Initialize BMX055 - IMU
	ton.setLED(RED);
	imu.init();
	ton.setLED(BLUE);

	// Configure function to be attached (flip) and the interval (0.1 second)
	flipper_1.attach(&flip_1, 0.1);

	// The main LOOP
	while (1)
	{
		usb.printf("pitch: %0.3f\r\n", imu.getPitch());
		usb.printf("roll: %0.3f\r\n", imu.getRoll());
		usb.printf("yaw: %0.3f\r\n", imu.getYaw());
		usb.printf("temp: %0.2f\r\n", imu.getTemperature());
		usb.printf("\r\n");

		ton.toggleLED(GREEN);
		wait(0.5);
	}   // end of main LOOP
}   // end of main function
