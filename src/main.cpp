#define F_CPU 7372800UL 
#include "UartAvr.h"
#include "Console.h"
#include "IWait.h"
#include "GpioAvrDriver.h"
#include "OneWireDriver.h"
#include "Ds18b20.h"
#include <avr/io.h>
#include <util/delay.h>

class AvrWait : public iwait::IWait {

public:
    virtual void wait_us(uint16_t time) {
        if (time == 2) {
            _delay_us(0.3);
            return;
        }

        if (time == 5) {
            _delay_us(1);
            return;
        }

        if (time == 10) {
            _delay_us(2);
            return;
        }

        if (time == 15) {
            _delay_us(3);
            return;
        }

        time = time >> 2;
        for (uint16_t i = 0; i < time; i++)
            _delay_us(2);
    };

    virtual void wait_ms(uint16_t time) {
        for (uint16_t i = 0; i < time; i++)
            _delay_ms(1);
    };
};


int main() {
    DDRC = 0xFF;
    PORTC = 0xFF;
    PINC = 0xFF;
    AvrWait avr_wait;
    uart_avr::UartAvr uart(uart_driver::BAUD_19200);

    gpio_avr_driver::GpioAvr one_wire_gpio(
            DDRC,
            PORTC,
            PINC,
            5,
            gpio_driver::GPIO_INPUT,
            gpio_driver::GPIO_NO_PULL);

    one_wire_driver::OneWireDriver one_wire(
            one_wire_gpio,
            avr_wait);

    ds18b20::Ds18b20 termo(
            one_wire,
            avr_wait);


    console::Console serial(uart);

    ds18b20::Temp temp;
    temp.value = 64;

    for(int i = 0; i < 100; i ++)
        serial.newline();


    while(1) {
        avr_wait.wait_ms(2000);

        temp = termo.GetTemp();
        serial.print("Temp=").print((int8_t)(temp.value >> 4))
            .print(".")
            .print((temp.value & 0x0F) * 6)
            .newline();
        for (int i = 0; i < 16; i++) {
            if (temp.value & (1 << (15 - i)))
                serial.print("1");
            else
                serial.print("0");
        }

        serial.newline();
    }

    return 0;
}
