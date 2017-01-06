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
#   include "ReadSettingsFromFile.h"
#   include <iomanip>
#   include <sstream>
#   include <unistd.h>
#   include <sys/time.h>
#   include <iostream>
#   include <stdlib.h>
#endif

#include <array>
#include "OneWireDriver.h"
#include "IWait.h"

#include "Ds18b20.h"

long _abs(long a) {

    if (a > 0)
        return a;
    else
        return -a;
}

#if defined(TARGET_PLATFORM_AVR)
class AvrWait : public iwait::IWait {

public:
    virtual void wait_us(uint16_t time) {
        if (time == 2) {
            _delay_us(0.3);
            return;
        }

        if (time == 4) {
            _delay_us(1);
            return;
        }

        time = time >> 1;
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

        timeval start, current;
        static int counter = 0;

        gettimeofday(&start, NULL);

        do
        {
            gettimeofday(&current, NULL);
        } while (_abs((current.tv_usec - start.tv_usec)%1000000) < time);
        return;
    }

    virtual void wait_ms(uint16_t time) {
        for (int i = 0; i < time; i++)
            this->wait_us(1000);
    }
};
#endif

std::array<int, 2> s = { 2, 5};

int main() {
#if defined(TARGET_PLATFORM_AVR)
    DDRC = 0xFF;
    PORTC = 0xFF;
    PINC = 0xFF;
#endif
    s[1] = 30;

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
    read_settings_from_file::ReadSettingsFromFile loader("/tmp/Transmitter/Settings.txt");
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

    int counter = 0;

    while(1) {
        wait.wait_ms(2000);

#if defined(TARGET_PLATFORM_AVR)
        temp = termo.GetTemp();
        console.print(counter++);
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
#endif

#if defined(TARGET_PLATFORM_RASPBERRYPI)
        ds18b20::Temp rpi_temp[5] = {{ 0 }};
        rpi_temp[0] = termo.GetTemp();
        rpi_temp[1] = termo.GetTemp();
        rpi_temp[2] = termo.GetTemp();
        rpi_temp[3] = termo.GetTemp();
        rpi_temp[4] = termo.GetTemp();

        for (int i = 0; i < 5; i++)
            for (int j = i + 1; j < 5; j++) {
                if (rpi_temp[i].value > rpi_temp[j].value) {
                    ds18b20::Temp tmp = rpi_temp[i];
                    rpi_temp[i] = rpi_temp[j];
                    rpi_temp[j] = tmp;
                }
            }

        float t = (float)rpi_temp[2].value / 16.0;
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << t;
//        std::cout << "temp(float) = " << t << std::endl;
        std::string curl_string = "curl \"";
        curl_string += loader.GetAsString(i_settings_loader::SETTING_TYPE_WEB_ADDRESS);
        curl_string += "?host=";
        curl_string += loader.GetAsString(i_settings_loader::SETTING_TYPE_HOST_NAME);
        curl_string += "&value=";
        curl_string += stream.str();
        curl_string += "\"";
//        std::cout << curl_string << std::endl;
        system(curl_string.c_str());
#endif


    }

    return 0;
}
