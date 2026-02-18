/** Example shows howto use fifo from irq context
 *  Hardware platform: STM32F411E-DISCO
 *  PA2 Port - USART TXD should be connected to serial<->usb converter
 */
#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>
#include <isix/arch/irq_platform.h>
#include <isix/arch/irq.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>

constexpr auto led_0 = periph::gpio::num::PD13;
constexpr auto btn_0 = periph::gpio::num::PA0;

static isix::fifo<bool> kolejeczka(10);

void t1(void*) {
    bool received_val;
	bool last_state = false;
	for (;;) {
		if (kolejeczka.pop(received_val, ISIX_TIME_INFINITE) == ISIX_EOK) {
			last_state = !last_state;
			periph::gpio::set(led_0, last_state);
            dbprintf("Odebrano informacje z kolejki! Zmiana LED na: %d", last_state);
		}
	}
}


void led_button_init() {
    periph::gpio::setup(led_0, periph::gpio::mode::out {
        periph::gpio::outtype::pushpull,
        periph::gpio::speed::low
    });

    periph::gpio::setup(btn_0, periph::gpio::mode::in { 
        periph::gpio::pulltype::down 
    });
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
    NVIC_SetPriority(EXTI0_IRQn, 6);
    NVIC_EnableIRQ(EXTI0_IRQn);
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

extern "C" {
	//! Exti 0 vector
	void exti0_isr_vector() {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
		kolejeczka.push_isr(true);
	}
}

// Start main function
auto main() -> int
{
    usart_protected_init();
    led_button_init();   
    
    dbprintf("<<<< START SYSTEMU KOLEJECZKA >>>>");

    static constexpr auto tsk_stack = 2048; 
    static constexpr auto tsk_prio = 6;

    isix::task_create(t1, nullptr, tsk_stack, tsk_prio);
	isix::start_scheduler();
    
    return 0;

}


