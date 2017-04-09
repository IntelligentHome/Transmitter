#if defined(TARGET_PLATFORM_AVR)
#   define F_CPU 7372800UL
#   include "GpioAvrDriver.h"
#   include "SpiAvr.h"
#   include "UartAvr.h"
#   include "Nrf24l01.h"
#   include <avr/io.h>
#   include <util/delay.h>
#   include <avr/interrupt.h>
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
    read_settings_from_file::ReadSettingsFromFile loader("/tmp/Transmitter/Settings.txt");
#endif

#if defined(TARGET_PLATFORM_AVR)
    console_avr::ConsoleAvr console(uart);
#elif defined(TARGET_PLATFORM_RASPBERRYPI)
    console_rpi::ConsoleRpi console;
#endif

#if defined(TARGET_PLATFORM_AVR)
    //sei();
    gpio_avr_driver::GpioAvr one_wire_gpio(
            DDRC,
            PORTC,
            PINC,
            5,
            gpio_driver::GPIO_INPUT,
            gpio_driver::GPIO_NO_PULL);

    //console.print("Init one_wire_gpio").newline();

    gpio_avr_driver::GpioAvr spi_gpio(
            DDRC,
            PORTC,
            PINC,
            2,
            gpio_driver::GPIO_OUTPUT,
            gpio_driver::GPIO_PULL_UP);

    //console.print("Init spi_gpio").newline();

    gpio_avr_driver::GpioAvr nrf24l01_ce(
            DDRC,
            PORTC,
            PINC,
            3,
            gpio_driver::GPIO_OUTPUT,
            gpio_driver::GPIO_PULL_UP);

    //console.print("Init nrf24l01_ce").newline();

    spi_avr::SpiAvr spi(&spi_gpio, &console);

    //console.print("Init spi").newline();

    nrf24l01_driver::Nrf24l01 radio(
            &spi,
            &nrf24l01_ce);

    radio.AddWaiter(&wait);

    radio.FlushRx();
    radio.FlushTx();
    radio.ClearIrqFlags();

    radio.StartListening();
    radio.StopListening();

    //console.print("Init radio").newline();

    nrf24_driver::NrfStatusRegister radio_status = {{ 0 }};

    {
        //TODO: Add support for command EN_RXADDR
        uint8_t data_debug[] = { 0x22, 0x03 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        //TODO: Add support for command SETUP_AW
        uint8_t data_debug[] = { 0x23, 0x03 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        //TODO: Add support for command REGISTER_RF_SETUO
        uint8_t data_debug[] = { 0x26, 0x0E };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        //TODO: Add support for Dynamic payload length
        uint8_t data_debug[] = { 0x3C, 0 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        //TODO: Add support for FEATURE
        uint8_t data_debug[] = { 0x3D, 0 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        uint8_t data_debug[] = { 0x2B, 0x00, 0x12, 0x23, 0x24, 0x24 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        uint8_t data_debug[] = { 0x2C, 0x55 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        uint8_t data_debug[] = { 0x2D, 0x66 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        uint8_t data_debug[] = { 0x2E, 0x77 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    {
        uint8_t data_debug[] = { 0x2F, 0x88 };
        radio.Send(data_debug, sizeof(data_debug));
    }

    console.print("Read regs").newline();
    {
        uint8_t data_debug[] = { 0x00, 0x00 };
        for (uint8_t ii = 0; ii <= 0x17; ii++) {
            data_debug[0] = ii;
            radio.Send(data_debug, sizeof(data_debug));
        }
    }

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


    ds18b20::Temp temp;
    temp.value = 64;

    for(int i = 0; i < 5; i ++)
        console.newline();

    console.print("----Start----").newline();

    int counter = 0;

    wait.wait_ms(10000);

    while(1) {
        wait.wait_ms(4000);

#if defined(TARGET_PLATFORM_AVR)
        radio_status = radio.GetStatus();
        console.print("radio_status = ").print(radio_status.raw_data).newline();
        console.print("tx_full=").print(radio_status.tx_full).newline();
        console.print("rx_pipe_no=").print(radio_status.rx_pipe_no).newline();
        console.print("max_rt=").print(radio_status.max_rt).newline();
        console.print("tx_data_ready=").print(radio_status.tx_data_ready).newline();
        console.print("rx_data_ready=").print(radio_status.rx_data_ready).newline();
        {
            uint8_t debug_data[] = { 0x17, 0x00 };
            radio.Send(debug_data, sizeof(debug_data));
        }

        if (radio_status.rx_data_ready) {
            radio.ClearIrqFlags();
            uint8_t payload_debug[32] = { 0 };

            radio.GetPayload(payload_debug, sizeof(payload_debug));
            console.print("Payload=").newline();
            for (int ii = 0; ii < 32; ii++)
                console.print(payload_debug[ii]).newline();
        }

        if (radio_status.tx_full || radio_status.max_rt) {
            console.print("tx_full, clearIrq").newline();
            radio.ClearIrqFlags();
            radio.FlushTx();
        }

        temp = termo.GetTemp();

        radio_status = radio.GetStatus();

        //Send data
        if (radio_status.tx_full == 0 && radio_status.max_rt == 0)
        {
//            const uint8_t tx_payload[] = {
//                0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
//            };

            console.print("Send message").newline();
            radio.SetPayload(temp.raw_data, sizeof(temp.raw_data));
            radio.SendData();
        }


        
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
