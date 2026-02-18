#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>

constexpr auto led_0 = periph::gpio::num::PD13;
constexpr auto btn_0 = periph::gpio::num::PA0;
static isix::semaphore semaforek{0, 1};


void t1(void*) {
    
    bool last_btn_state = false;

    for(;;) {
        bool curr_btn_state = periph::gpio::get(btn_0);

        if (curr_btn_state && !last_btn_state) {
            
            isix::wait_ms(20);
            
            if (periph::gpio::get(btn_0)) {
                dbprintf("Przycisk wcisniety!");
                bool curr_val = semaforek.getval();
                if (curr_val) {
                    dbprintf("Dioda ma sie wylaczyc!");
                    semaforek.reset(0);
                }
                else {
                    dbprintf("Dioda ma sie wlaczyc!");
                    semaforek.signal();
                }
            }
        }

        last_btn_state = curr_btn_state;

        isix::wait_ms(20); 
    }
}


void t2(void*) {
    for(;;) {
        periph::gpio::set(led_0, semaforek.getval());
        isix::wait_ms(20); 
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
    
    dbprintf("<<<< START SYSTEMU SEMAFOREK >>>>");

    static constexpr auto tsk_stack = 2048; 
    static constexpr auto tsk_prio = 6;

    isix::task_create(t1, nullptr, tsk_stack, tsk_prio);
    isix::task_create(t2, nullptr, tsk_stack, tsk_prio);


    isix::start_scheduler();
    
    return 0;
}