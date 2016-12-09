#include <avr/io.h>
#include "Nrf24l01.h"

int main() {

    nrf24l01_driver::Nrf24l01 nrf(
            0,
            0);

    while(1)
    {
        nrf.FlushRx();
        nrf.FlushTx();
    }

    return 0;
}
