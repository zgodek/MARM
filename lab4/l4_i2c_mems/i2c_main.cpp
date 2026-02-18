#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/drivers/i2c/i2c_master.hpp>
#include <isix.h>
#include <cstring>
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

namespace {
    static constexpr int LSM303_ACC_ADDR = 0x32;

	static constexpr uint8_t CTRL_REG1_A = 0x20;
	static constexpr uint8_t OUT_X_L_A   = 0x28; 
	static constexpr uint8_t AUTO_INCREMENT = 0x80;

    // periph::drivers::i2c_master* i2c = nullptr;

    // int i2c_init_isix() {
	// 	i2c = new periph::drivers::i2c_master("i2c1");
	// 	static constexpr auto IFC_TIMEOUT = 1000;
	// 	int ret = i2c->open(IFC_TIMEOUT);
	// 	if(ret) {
	// 		dbg_err("Unable to open i2c device error: %i", ret);
	// 		return ret;
	// 	}
	// 	ret = i2c->set_option(periph::option::speed(400'000));
	// 	if(ret) {
	// 		dbg_err("Unable to set i2c option error: %i", ret);
	// 		return ret;
	// 	}
	// 	return 0;
    // }
    
    // int i2c_write_reg_isix(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    //     uint8_t buf[] = { reg_addr, value };
    //     periph::blk::tx_transfer tran(buf, sizeof(buf));
    //     return i2c->transaction(dev_addr, tran);
    // }

    // int i2c_read_regs_isix(uint8_t dev_addr, uint8_t reg_addr, uint8_t* rx_buffer, size_t len) {
    //     uint8_t tx_buf[] = { reg_addr };
    //     periph::blk::trx_transfer tran(tx_buf, rx_buffer, sizeof(tx_buf), len);
    //     return i2c->transaction(dev_addr, tran);
    // }

    // int lsm303_init() {
    //     return i2c_write_reg_isix(LSM303_ACC_ADDR, CTRL_REG1_A, 0x27);
    // }

    // int lsm303_read_accel(int16_t& x, int16_t& y, int16_t& z) {
    //     uint8_t buf[6] = {}; 
    //     int ret = i2c_read_regs_isix(LSM303_ACC_ADDR, OUT_X_L_A | AUTO_INCREMENT, buf, 6);
    //     if (ret != 0) return ret;

    //     x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
    //     y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    //     z = static_cast<int16_t>((buf[5] << 8) | buf[4]);
    //     return 0;
    // }

	#define I2C_INSTANCE    I2C1
    #define I2C_GPIO_PORT   GPIOB
    #define I2C_SCL_PIN     LL_GPIO_PIN_6
    #define I2C_SDA_PIN     LL_GPIO_PIN_9
    #define I2C_AF          LL_GPIO_AF_4


    int i2c_init_ll() {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

        LL_I2C_InitTypeDef I2C_InitStruct;
        LL_GPIO_InitTypeDef GPIO_InitStruct;

        // Konfiguracja GPIO (Open-Drain i Pull-Up)
        LL_GPIO_StructInit(&GPIO_InitStruct);
        GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; // Wewnętrzne pull-upy
        GPIO_InitStruct.Alternate = I2C_AF;
        LL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

        // Konfiguracja I2C
        LL_I2C_Disable(I2C_INSTANCE);
        LL_I2C_StructInit(&I2C_InitStruct);
        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.ClockSpeed = 400000;
        I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        
        if (LL_I2C_Init(I2C_INSTANCE, &I2C_InitStruct) != SUCCESS) {
            return -1;
        }
        
        LL_I2C_Enable(I2C_INSTANCE);
        return 0;
    }

    int i2c_write_reg_ll(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
        // 1. START
        LL_I2C_GenerateStartCondition(I2C_INSTANCE);
        while (!LL_I2C_IsActiveFlag_SB(I2C_INSTANCE));

        // 2. Adres Urządzenia + zapis
        LL_I2C_TransmitData8(I2C_INSTANCE, dev_addr);
        while (!LL_I2C_IsActiveFlag_ADDR(I2C_INSTANCE));
        LL_I2C_ClearFlag_ADDR(I2C_INSTANCE);

        // 3. Adres Rejestru
        while (!LL_I2C_IsActiveFlag_TXE(I2C_INSTANCE));
        LL_I2C_TransmitData8(I2C_INSTANCE, reg_addr);

        // 4. Wartość
        while (!LL_I2C_IsActiveFlag_TXE(I2C_INSTANCE));
        LL_I2C_TransmitData8(I2C_INSTANCE, value);

        while (!LL_I2C_IsActiveFlag_BTF(I2C_INSTANCE));
        
        // 6. STOP
        LL_I2C_GenerateStopCondition(I2C_INSTANCE);
        
        return 0;
    }

    int i2c_read_regs_ll(uint8_t dev_addr, uint8_t reg_addr, uint8_t* rx_buffer, size_t len) {
        // 1. START
        LL_I2C_GenerateStartCondition(I2C_INSTANCE);
        while (!LL_I2C_IsActiveFlag_SB(I2C_INSTANCE));

        // 2. Adres Urządzenia + zapis
        LL_I2C_TransmitData8(I2C_INSTANCE, dev_addr);
        while (!LL_I2C_IsActiveFlag_ADDR(I2C_INSTANCE));
        LL_I2C_ClearFlag_ADDR(I2C_INSTANCE);

        // 3. Adres Rejestru
        while (!LL_I2C_IsActiveFlag_TXE(I2C_INSTANCE));
        LL_I2C_TransmitData8(I2C_INSTANCE, reg_addr);
        
        while (!LL_I2C_IsActiveFlag_TXE(I2C_INSTANCE)); 

        // 4. REPEATED START
        LL_I2C_GenerateStartCondition(I2C_INSTANCE);
        while (!LL_I2C_IsActiveFlag_SB(I2C_INSTANCE));

        // 5. Adres Urządzenia + odczyt
        LL_I2C_TransmitData8(I2C_INSTANCE, dev_addr | 0x01);
        while (!LL_I2C_IsActiveFlag_ADDR(I2C_INSTANCE));
        
        if (len == 1) {
            // Jeśli tylko 1 bajt:
            LL_I2C_AcknowledgeNextData(I2C_INSTANCE, LL_I2C_NACK);
            LL_I2C_ClearFlag_ADDR(I2C_INSTANCE);
            LL_I2C_GenerateStopCondition(I2C_INSTANCE); // STOP od razu po ADDR
        } else {
            // Jeśli więcej niż 1 bajt:
            LL_I2C_AcknowledgeNextData(I2C_INSTANCE, LL_I2C_ACK);
            LL_I2C_ClearFlag_ADDR(I2C_INSTANCE);
        }

        // 6. Pętla odczytu danych
        for (size_t i = 0; i < len; i++) {
            if (i == len - 1) {
				//Jeśli to już koniec danych to trzeba wysłać NACK i STOP
                if (len > 1) {
                    LL_I2C_AcknowledgeNextData(I2C_INSTANCE, LL_I2C_NACK);
                    LL_I2C_GenerateStopCondition(I2C_INSTANCE);
                }
            }

            while (!LL_I2C_IsActiveFlag_RXNE(I2C_INSTANCE));
            
            // Odczyt danych
            rx_buffer[i] = LL_I2C_ReceiveData8(I2C_INSTANCE);
        }

        return 0;
    }

    int lsm303_init_ll() {
        return i2c_write_reg_ll(LSM303_ACC_ADDR, CTRL_REG1_A, 0x27);
    }

    int lsm303_read_accel_ll(int16_t& x, int16_t& y, int16_t& z) {
        uint8_t buf[6] = {}; 
        
        int ret = i2c_read_regs_ll(LSM303_ACC_ADDR, OUT_X_L_A | AUTO_INCREMENT, buf, 6);
        if (ret != 0) return ret;

        x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
        y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
        z = static_cast<int16_t>((buf[5] << 8) | buf[4]);
        return 0;
    }

    void watch_sensor(void*) {
        int ret = 0;

        if ((ret = i2c_init_ll()) != 0) {
            dbg_err("I2C init failed: %i", ret);
            return;
        }

        if ((ret = lsm303_init_ll()) != 0) {
            dbg_err("LSM303 init failed: %i", ret);
        } else {
            dbprintf("LSM303 Init OK");
        }

        while (true) {
            int16_t x, y, z;
            if (lsm303_read_accel_ll(x, y, z) == 0) {
                dbprintf("ACC X:%6d Y:%6d Z:%6d", x, y, z);
            }
            isix::wait_ms(500);
        }
    }

    auto usart_protected_init() -> void {
        static isix::mutex m_mtx;
        dblog_init_locked(
                [](int ch, void*) { return periph::drivers::uart_early::putc(ch); },
                nullptr,
                []() { m_mtx.lock();  },
                []() { m_mtx.unlock(); },
                periph::drivers::uart_early::open, "serial0", 115200
        );
    }
}

auto main() -> int
{
    usart_protected_init();
    
    isix::wait_ms(500);
    
    static constexpr auto tsk_stack = 2048; 
    static constexpr auto tsk_prio = 6;
    
    dbprintf("<<<< MEMS I2C LSM303 Demo >>>>");
    
    isix::task_create(watch_sensor, nullptr, tsk_stack, tsk_prio);
    isix::start_scheduler();
    return 0;
}