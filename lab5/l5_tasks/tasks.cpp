#include <config/conf.h>
#include <periph/gpio/gpio.hpp>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <isix.h>

constexpr auto led_0 = periph::gpio::num::PD13;
constexpr auto btn_0 = periph::gpio::num::PA0;


volatile ostask_t t2_handle = nullptr;

void t1(void*) {
    bool t2_is_running = false;
    
    bool last_btn_state = false;

    for(;;) {
        bool curr_btn_state = periph::gpio::get(btn_0);

        if (curr_btn_state && !last_btn_state) {
            
            isix::wait_ms(20);
            
            if (periph::gpio::get(btn_0)) {
                dbprintf("Przycisk wcisniety!");

                if (t2_handle != nullptr) {
                    if (t2_is_running) {
                        dbprintf("Usypiam T2");
                        isix_task_suspend(t2_handle);
                        t2_is_running = false;
                    } else {
                        dbprintf("Budze T2");
                        isix_task_resume(t2_handle);
                        t2_is_running = true;
                    }
                }
            }
        }

        last_btn_state = curr_btn_state;

        isix::wait_ms(20); 
    }
}

void t2(void*) {
    int i = 0;
    for(;;) {
        periph::gpio::set(led_0, i % 2);
        i++;
        isix::wait_ms(250); 
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
    
    dbprintf("<<<< START SYSTEMU BUDZIK >>>>");

    static constexpr auto tsk_stack = 2048; 
    static constexpr auto tsk_prio = 6;
    
    static auto t2_thread_obj = isix::thread_create_and_run(
        tsk_stack, 
        tsk_prio, 
        isix_task_flag_suspended,
        t2, 
        nullptr
    );
    
    t2_handle = t2_thread_obj.tid();

    if (t2_handle == nullptr) {
        dbprintf("Blad tworzenia watku T2!");
    }

    isix::task_create(t1, nullptr, tsk_stack, tsk_prio);

    isix::start_scheduler();
    
    return 0;
}