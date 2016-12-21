#if defined(TARGET_PLATFORM_AVR)
#   define F_CPU 7372800UL
#   include "GpioAvrDriver.h"
#   include "UartAvr.h"
#   include <avr/io.h>
#   include <util/delay.h>
#   include "ConsoleAvr.h"
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
#   include "GpioRpi.h"
#   include "ConsoleRpi.h"
#   include <unistd.h>
#endif

#include "OneWireDriver.h"
#include "IWait.h"

#include "Ds18b20.h"

#if defined(TARGET_PLATFORM_AVR)
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
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
class RpiWait : public iwait::IWait {

public:
    virtual void wait_us(uint16_t time) {
        usleep(time);    
    }

    virtual void wait_ms(uint16_t time) {
        usleep(time * 1000);
    }
};
#endif


int main() {
#if defined(TARGET_PLATFORM_AVR)
    DDRC = 0xFF;
    PORTC = 0xFF;
    PINC = 0xFF;
#endif

#if defined(TARGET_PLATFORM_AVR)
#   pragma message ("Platform AVR")
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
#   pragma message ("Platform Raspberry pi")
#endif

#if defined(TARGET_PLATFORM_AVR)
    AvrWait wait;
    uart_avr::UartAvr uart(uart_driver::BAUD_19200);
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
    RpiWait wait;
#endif

#if defined(TARGET_PLATFORM_AVR)
    gpio_avr_driver::GpioAvr one_wire_gpio(
            DDRC,
            PORTC,
            PINC,
            5,
            gpio_driver::GPIO_INPUT,
            gpio_driver::GPIO_NO_PULL);

#elif defined(TARGET_PLATFORM_RASPBERRYPI)
    gpio_rpi::GpioRpi one_wire_gpio(
            17,
            gpio_driver::GPIO_INPUT,
            gpio_driver::GPIO_NO_PULL);
#endif

    one_wire_driver::OneWireDriver one_wire(
            one_wire_gpio,
            wait);

    ds18b20::Ds18b20 termo(
            one_wire,
            wait);

#if defined(TARGET_PLATFORM_AVR)
    console_avr::ConsoleAvr console(uart);
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
    console_rpi::ConsoleRpi console;
#endif

    ds18b20::Temp temp;
    temp.value = 64;

    for(int i = 0; i < 5; i ++)
        console.newline();

    console.print("----Start----").newline();


    while(1) {
        wait.wait_ms(2000);

        temp = termo.GetTemp();
        console.print("Temp=").print((int8_t)(temp.value >> 4))
            .print(".")
            .print((temp.value & 0x0F) * 6)
            .newline();
        for (int i = 0; i < 16; i++) {
            if (temp.value & (1 << (15 - i)))
                console.print("1");
            else
                console.print("0");
        }

        console.newline();
    }

    return 0;
}
