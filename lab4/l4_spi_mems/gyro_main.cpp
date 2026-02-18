#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/drivers/spi/spi_master.hpp>
#include <isix.h>
#include <periph/dma/dma.hpp>
#include <cstring>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"

namespace {
    constexpr auto led_green  = periph::gpio::num::PD12; // LD4
    constexpr auto led_orange = periph::gpio::num::PD13; // LD3
    constexpr auto led_red    = periph::gpio::num::PD14; // LD5
    constexpr auto led_blue   = periph::gpio::num::PD15; // LD6

    constexpr int16_t THR = 500; // wartość bezwgzl odchylenia
    constexpr int CS_ADDR = 0;

    struct reg {
        static constexpr uint8_t WHO_AM_I      = 0x0F;
        static constexpr uint8_t CTRL_REG1     = 0x20;
        static constexpr uint8_t CTRL_REG2     = 0x21;
        static constexpr uint8_t CTRL_REG3     = 0x22;
        static constexpr uint8_t CTRL_REG4     = 0x23;
        static constexpr uint8_t CTRL_REG5     = 0x24;
        static constexpr uint8_t OUT_X_L       = 0x28;
        static constexpr uint8_t OUT_X_H       = 0x29;
        static constexpr uint8_t OUT_Y_L       = 0x2A;
        static constexpr uint8_t OUT_Y_H       = 0x2B;
        static constexpr uint8_t OUT_Z_L       = 0x2C;
        static constexpr uint8_t OUT_Z_H       = 0x2D;
        
    };

    // periph::drivers::spi_master* spi = nullptr;

    // int spi_init_isix() {
    //     namespace opt = periph::option;
    //     int ret = 0;
    //     if (spi == nullptr) {
    //         spi = new periph::drivers::spi_master("spi1");
    //     }
    //     do {
    //         // 8 bitów danych
    //         if((ret=spi->set_option(opt::dwidth(8)))<0) break;
	// 	    // Pierwszy bit najbardziej znaczący (MSB)
    //         if((ret=spi->set_option(opt::bitorder(
    //                                opt::bitorder::msb)))<0) break;
	// 	    // Częstotliwość 10Mhz
    //         if((ret=spi->set_option(opt::speed(10E6)))<0) break;
    //         // Próbkowanie danych na pierwszym zboczu zegara
    //         if((ret=spi->set_option(opt::phase(
    //                                opt::phase::_1_edge)))<0) break;
	// 	    // Próbkowanie danych przy zmianie z niskiego na wysoki
    //         if((ret=spi->set_option(opt::polarity(
    //                                opt::polarity::low)))<0) break;
    //         if((ret=spi->open(ISIX_TIME_INFINITE))<0) break;
    //    } while(0);
    //    return ret;

    // }

    // int spi_write_reg_isix(uint8_t addr, uint8_t val) {
    //     uint8_t buf[] = { addr, val }; 
    //     periph::blk::tx_transfer tran(buf, sizeof(buf));
    //     return spi->transaction(CS_ADDR, tran);
    // }

    // int spi_read_reg_isix(uint8_t addr, uint8_t& val) {
    //     uint8_t tx[] = { static_cast<uint8_t>(addr | 0x80), 0x00 };
    //     uint8_t rx[2] = {};
        
    //     periph::blk::trx_transfer tran(tx, rx, sizeof(tx));
    //     int ret = spi->transaction(CS_ADDR, tran);
        
    //     if (ret == 0) {
    //         val = rx[1];
    //     }
    //     return ret;
    // }

    // int l3gd20_init_isix() {
    //     uint8_t id = 0;
        
    //     if (spi_read_reg_isix(reg::WHO_AM_I, id) != 0) {
    //         return -1;
    //     }
    //     dbprintf("L3GD20 ID: 0x%02X", id);
        
    //     if (id != 0xD4 && id != 0xD3 && id != 0xD7) {
    //         return -2; 
    //     }

    //     int ret = 0;
    //     ret |= spi_write_reg_isix(reg::CTRL_REG1, 0x0F);
    //     ret |= spi_write_reg_isix(reg::CTRL_REG2, 0x00);
    //     ret |= spi_write_reg_isix(reg::CTRL_REG3, 0b00001000); 
    //     ret |= spi_write_reg_isix(reg::CTRL_REG4, 0b00110000);
    //     ret |= spi_write_reg_isix(reg::CTRL_REG5, 0x00);
    //     return ret;
    // }

    // int l3gd20_read_all_isix(int16_t& x, int16_t& y, int16_t& z) {
    //     uint8_t xl, xh, yl, yh, zl, zh;
    //     int ret = 0;

    //     ret |= spi_read_reg_isix(reg::OUT_X_L, xl);
    //     ret |= spi_read_reg_isix(reg::OUT_X_H, xh);
    //     ret |= spi_read_reg_isix(reg::OUT_Y_L, yl);
    //     ret |= spi_read_reg_isix(reg::OUT_Y_H, yh);
    //     ret |= spi_read_reg_isix(reg::OUT_Z_L, zl);
    //     ret |= spi_read_reg_isix(reg::OUT_Z_H, zh);

    //     if(ret != 0) return ret;

    //     x = static_cast<int16_t>((xh << 8) | xl);
    //     y = static_cast<int16_t>((yh << 8) | yl);
    //     z = static_cast<int16_t>((zh << 8) | zl);
    //     return 0;
    // }


    
    #define SPI_SCK_PIN   LL_GPIO_PIN_5
    #define SPI_MISO_PIN  LL_GPIO_PIN_6
    #define SPI_MOSI_PIN  LL_GPIO_PIN_7

    int spi_init_ll() {
        //Włączenie zegarów
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

        //Inicjalizacja struktur
        LL_SPI_InitTypeDef SPI_InitStruct;
        LL_GPIO_InitTypeDef GPIO_InitStruct;

        LL_SPI_StructInit(&SPI_InitStruct);
        LL_GPIO_StructInit(&GPIO_InitStruct);

        //Konfiguracja pinu CS (PE3) jako Output Push-Pull
        LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3); // CS domyślnie w stanie wysokim
        
        GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        //Konfiguracja pinów SPI (PA5, PA6, PA7) jako Alternate Function        
        GPIO_InitStruct.Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Konfiguracja parametrów SPI zgodnie z instrukcją
        SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
        SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
        SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
        SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
        SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
        SPI_InitStruct.NSS = LL_SPI_NSS_SOFT; //NSS wyłączony, 'ręczne' sterowanie CS
        SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16; //zegar poniżej 10 Mhz (100/16 = 6,25 Mhz)
        SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
        SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
        
        if (LL_SPI_Init(SPI1, &SPI_InitStruct) != SUCCESS) {
            return -1;
        }

        LL_SPI_Enable(SPI1);
        
        return 0;
    }

    uint8_t spi_ll_transfer_byte(uint8_t data) {
        while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {}
        
        LL_SPI_TransmitData8(SPI1, data);
        
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
        
        return LL_SPI_ReceiveData8(SPI1);
    }

    int spi_write_reg_ll(uint8_t addr, uint8_t val) {
        LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_3);
        
        spi_ll_transfer_byte(addr);
        spi_ll_transfer_byte(val);

        LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3);
        return 0;
    }

    int spi_read_reg_ll(uint8_t addr, uint8_t& val) {
        LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_3);
        
        spi_ll_transfer_byte(addr | 0x80);
        val = spi_ll_transfer_byte(0x00);
        
        LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_3);
        return 0;
    }

    int l3gd20_init_ll() {
        uint8_t id = 0;
        
        spi_read_reg_ll(reg::WHO_AM_I, id);
        dbprintf("L3GD20 ID: 0x%02X", id);
        
        if (id != 0xD4 && id != 0xD3 && id != 0xD7) {
            return -2; 
        }

        spi_write_reg_ll(reg::CTRL_REG1, 0x0F);
        spi_write_reg_ll(reg::CTRL_REG2, 0x00);
        spi_write_reg_ll(reg::CTRL_REG3, 0b00001000); 
        spi_write_reg_ll(reg::CTRL_REG4, 0b00110000);
        spi_write_reg_ll(reg::CTRL_REG5, 0x00);
        return 0;
    }

    int l3gd20_read_all_ll(int16_t& x, int16_t& y, int16_t& z) {
        uint8_t xl, xh, yl, yh, zl, zh;

        spi_read_reg_ll(reg::OUT_X_L, xl);
        spi_read_reg_ll(reg::OUT_X_H, xh);
        spi_read_reg_ll(reg::OUT_Y_L, yl);
        spi_read_reg_ll(reg::OUT_Y_H, yh);
        spi_read_reg_ll(reg::OUT_Z_L, zl);
        spi_read_reg_ll(reg::OUT_Z_H, zh);

        x = static_cast<int16_t>((xh << 8) | xl);
        y = static_cast<int16_t>((yh << 8) | yl);
        z = static_cast<int16_t>((zh << 8) | zl);
        return 0;
    }


    void leds_init() {
        const auto mode_out = periph::gpio::mode::out {
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        };

        periph::gpio::setup(led_green,  mode_out);
        periph::gpio::setup(led_orange, mode_out);
        periph::gpio::setup(led_red,    mode_out);
        periph::gpio::setup(led_blue,   mode_out);
    }

    void leds_update(int16_t x, int16_t y) {

        // Oś X: Green (PD12) / Red (PD14)
        if (x > THR) {
            periph::gpio::set(led_green, true);
            periph::gpio::set(led_red,   false);
        } else if (x < -THR) {
            periph::gpio::set(led_green, false);
            periph::gpio::set(led_red,   true);
        } else {
            periph::gpio::set(led_green, false);
            periph::gpio::set(led_red,   false);
        }

        // Oś Y: Orange (PD13) / Blue (PD15)
        if (y > THR) {
            periph::gpio::set(led_blue,   true);
            periph::gpio::set(led_orange, false);
        } else if (y < -THR) {
            periph::gpio::set(led_blue,   false);
            periph::gpio::set(led_orange, true);
        } else {
            periph::gpio::set(led_blue,   false);
            periph::gpio::set(led_orange, false);
        }
    }

    static isix::semaphore m_ulock_sem { 1, 1 };

    void usart_protected_init() {
        dblog_init_locked(
            [](int ch, void*) {
                return periph::drivers::uart_early::putc(ch);
            },
            nullptr,
            []() {
                m_ulock_sem.wait(ISIX_TIME_INFINITE);
            },
            []() {
                m_ulock_sem.signal();
            },
            periph::drivers::uart_early::open, "serial0", 115200
        );
    }

    void watch_sensor(void*) {
        int ret {};
        
        if ((ret = spi_init_ll()) < 0) {
            dbprintf("SPI init failed: %d", ret);
            return;
        }

        if ((ret = l3gd20_init_ll()) != 0) {
            dbprintf("Gyro init failed: %d", ret);
            return;
        }

        dbprintf("SPI and Gyro init OK. Loop start.");

        do {
            while(true) {
                int16_t x, y, z;
                if (l3gd20_read_all_ll(x, y, z) == 0) {
                    dbprintf("X:%5d Y:%5d Z:%5d", x, y, z);
                    leds_update(x, y);
                } else {
                    dbprintf("Error reading axes");
                }
                isix::wait_ms(250);
            }

        } while(0);

        dbg_info("Task failed finished with code %i", ret);
    }
}

auto main() -> int
{
    isix::wait_ms(500);
    leds_init();    
    usart_protected_init();
    static constexpr auto tsk_stack = 1024;
    static constexpr auto tsk_prio = 6;
    
    isix::task_create(watch_sensor, nullptr, tsk_stack, tsk_prio);
    
    dbprintf("<<<< MEMS L3GD20 SPI Demo >>>>");
    isix::start_scheduler();
    return 0;
}