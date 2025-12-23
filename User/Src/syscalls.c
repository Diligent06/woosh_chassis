#include <sys/unistd.h> // write()
#include "usbd_cdc_if.h"

extern volatile uint8_t cdc_tx_busy;

int _write(int file, char *ptr, int len)
{
    CDC_Transmit_HS((uint8_t*)ptr, len);
    return len;
}
