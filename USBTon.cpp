#include "stdint.h"
#include "USBTon.h"

int USBTon::_putc(int c) {
    VCP_write((uint8_t *)&c, 1);
    return 1;
}

int USBTon::_getc() {
    uint8_t c = 0;
    VCP_read((uint8_t *)&c, 1);
    return c;
}

uint8_t USBTon::available() {
    return 1;
}
