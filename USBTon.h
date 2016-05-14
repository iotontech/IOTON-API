#ifndef USBTON_H
#define USBTON_H

#include "Stream.h"
#include "usb_user.h"
#include "USBCDC.h"

/**
* USBTon example
*
* @code
* #include "mbed.h"
* #include "USBTon.h"
*
* //Virtual serial port over USB
* USBTon usb;
*
* int main(void) {
*
*    while(1)
*    {
*        usb.printf("I am a virtual serial port\n");
*        wait(1);
*    }
* }
* @endcode
*/
class USBTon: public Stream {
public:

    /**
    *   Constructor
    *
    * @param vendor_id Your vendor_id (default: 0x1f00)
    * @param product_id Your product_id (default: 0x2012)
    * @param product_release Your preoduct_release (default: 0x0001)
    * @param connect_blocking define if the connection must be blocked if USB not plugged in
    *
    */
    USBTon(uint16_t vendor_id = 0x1f00, uint16_t product_id = 0x2012, uint16_t product_release = 0x0001, bool connect_blocking = true) {

    };


    /**
    * Send a character. You can use puts, printf.
    *
    * @param c character to be sent
    * @returns true if there is no error, false otherwise
    */
    virtual int _putc(int c);

    /**
    * Read a character: blocking
    *
    * @returns character read
    */
    virtual int _getc();

    /**
    * Check the number of bytes available.
    *
    * @returns the number of bytes available
    */
    uint8_t available();

    /** Determine if there is a character available to read
     *
     *  @returns
     *    1 if there is a character available to read,
     *    0 otherwise
     */
    int readable() { return available() ? 1 : 0; }

    /** Determine if there is space available to write a character
     *
     *  @returns
     *    1 if there is space to write a character,
     *    0 otherwise
     */
    int writeable() { return 1; } // always return 1, for write operation is blocking

};

#endif
