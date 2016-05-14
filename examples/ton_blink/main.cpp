// Includes --------------------------------------------------------------------
#include "Ioton.h"


// *****************************************************************************
// MAIN PROGRAM ****************************************************************
int main(void)
{
	// The main LOOP
	while (1)
	{
		ton.setLED(RED);
		wait(0.5);
		ton.setLED(GREEN);
		wait(0.5);
		ton.setLED(BLUE);
		wait(0.5);
		ton.setLED(YELLOW);
		wait(0.5);
		ton.setLED(CYAN);
		wait(0.5);
		ton.setLED(MAGENTA);
		wait(0.5);
		ton.setLED(WHITE);
		wait(0.5);
		ton.setLED(NONE);
		wait(0.5);
		ton.setLED("00AFEF");	// Ioton color
		wait(0.5);
	}   // end of main LOOP
}   // end of main function
