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

constexpr auto led_3 = periph::gpio::num::PD13;
constexpr auto led_4 = periph::gpio::num::PD12;
constexpr auto led_5 = periph::gpio::num::PD14;
constexpr auto led_6 = periph::gpio::num::PD15;
constexpr auto k0 = periph::gpio::num::PC0;
constexpr auto k1 = periph::gpio::num::PC1;
constexpr auto k2 = periph::gpio::num::PC2;
constexpr auto k3 = periph::gpio::num::PC3;
constexpr auto k4 = periph::gpio::num::PC4;


static isix::event wydarzenia;

void tk0(void*) {
    bool last_k0 = true;
	for (;;) {
        bool curr_k0 = periph::gpio::get(k0);
        if(!curr_k0 && last_k0) {
            dbprintf("Wykryto stan wysoki na linii PC0");
            wydarzenia.set(1);
        }
        last_k0 = curr_k0;
		isix::wait_ms(20);
	}
}

void tk1(void*) {
    bool last_k1 = true;
	for (;;) {
        bool curr_k1 = periph::gpio::get(k1);
        if(!curr_k1 && last_k1) {
            dbprintf("Wykryto stan wysoki na linii PC1");
            wydarzenia.set(2);
        }
        last_k1 = curr_k1;
		isix::wait_ms(20);
	}
}

void tk2(void*) {
    bool last_k2 = true;
	for (;;) {
        bool curr_k2 = periph::gpio::get(k2);
        if(!curr_k2 && last_k2) {
            dbprintf("Wykryto stan wysoki na linii PC2");
            wydarzenia.set(4);
        }
        last_k2 = curr_k2;
		isix::wait_ms(20);
	}
}

void tk3(void*) {
    bool last_k3 = true;
	for (;;) {
        bool curr_k3 = periph::gpio::get(k3);
        if(!curr_k3 && last_k3) {
            dbprintf("Wykryto stan wysoki na linii PC3");
            wydarzenia.set(8);
        }
        last_k3 = curr_k3;
		isix::wait_ms(20);
	}
}

void tk4(void*) {
    bool last_k4 = true;
	for (;;) {
        bool curr_k4 = periph::gpio::get(k4);
        if(!curr_k4 && last_k4) {
            dbprintf("Wykryto stan wysoki na linii PC4");
            wydarzenia.set(16);
        }
        last_k4 = curr_k4;
		isix::wait_ms(20);
	}
} 


void tl3(void*) {
    bool last_state = false;
	for (;;) {
        if (wydarzenia.wait(1, true, true, ISIX_TIME_INFINITE) & 1) {
            last_state = !last_state;
            if (last_state) {
                dbprintf("Włączam diodę LD3");
            }
            else {
                dbprintf("Wyłączam diodę LD3");

            }
            periph::gpio::set(led_3, last_state);
        }
	}
}

void tl4(void*) {
    bool last_state = false;
	for (;;) {
        if (wydarzenia.wait(2, true, true, ISIX_TIME_INFINITE) & 2) {
            last_state = !last_state;
            if (last_state) {
                dbprintf("Włączam diodę LD4");
            }
            else {
                dbprintf("Wyłączam diodę LD4");

            }
            periph::gpio::set(led_4, last_state);
        }
	}
}

void tl5(void*) {
    bool last_state = false;
	for (;;) {
        if (wydarzenia.wait(4, true, true, ISIX_TIME_INFINITE) & 4) {
            last_state = !last_state;
            if (last_state) {
                dbprintf("Włączam diodę LD5");
            }
            else {
                dbprintf("Wyłączam diodę LD5");

            }
            periph::gpio::set(led_5, last_state);
        }
	}
}

void tl6(void*) {
    bool last_state = false;
	for (;;) {
        if (wydarzenia.wait(24, true, true, ISIX_TIME_INFINITE) & 24) {
            last_state = !last_state;
            if (last_state) {
                dbprintf("Włączam diodę LD6");
            }
            else {
                dbprintf("Wyłączam diodę LD6");

            }
            periph::gpio::set(led_6, last_state);
        }
	}
}


void led_button_init() {
    periph::gpio::setup(led_3, periph::gpio::mode::out {
        periph::gpio::outtype::pushpull,
        periph::gpio::speed::low
    });
    periph::gpio::setup(led_4, periph::gpio::mode::out {
        periph::gpio::outtype::pushpull,
        periph::gpio::speed::low
    });
    periph::gpio::setup(led_5, periph::gpio::mode::out {
        periph::gpio::outtype::pushpull,
        periph::gpio::speed::low
    });
    periph::gpio::setup(led_6, periph::gpio::mode::out {
        periph::gpio::outtype::pushpull,
        periph::gpio::speed::low
    });
	periph::gpio::setup(k0, periph::gpio::mode::in {
    	periph::gpio::pulltype::up
	});
    periph::gpio::setup(k1, periph::gpio::mode::in {
    	periph::gpio::pulltype::up
	});
	periph::gpio::setup(k2, periph::gpio::mode::in {
    	periph::gpio::pulltype::up
	});
	periph::gpio::setup(k3, periph::gpio::mode::in {
    	periph::gpio::pulltype::up
	});
    periph::gpio::setup(k4, periph::gpio::mode::in {
    	periph::gpio::pulltype::up
	});
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


auto main() -> int
{
    usart_protected_init();
    led_button_init();   
    
    dbprintf("<<<< START SYSTEMU WYDARZENIA >>>>");

    static constexpr auto tsk_stack = 2048; 
    static constexpr auto tsk_prio = 6;

    isix::task_create(tk0, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tk1, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tk2, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tk3, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tk4, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tl3, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tl4, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tl5, nullptr, tsk_stack, tsk_prio);
    isix::task_create(tl6, nullptr, tsk_stack, tsk_prio);

    isix::start_scheduler();
    
    return 0;
}
