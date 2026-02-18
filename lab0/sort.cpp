#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <math.h>


namespace {
    constexpr auto led_0 = periph::gpio::num::PD13;
}

namespace app {
    void selection_sort(void*) {
        constexpr int N = 1000;
        constexpr int measurements = 10;

        std::array<uint32_t, measurements> times_us;

        static volatile int data[N];

        for(int m = 0; m < measurements; ++m) {
            uint32_t x = 1234 + m*7;
            for(int i = 0; i < N; ++i) {
                x = 1664525 * x + 1013904223;
                data[i] = x % N;
            }

            dbprintf("Array generated.\n");

            dbprintf("Let's sort this out...\n");
            auto start_time = isix::get_ujiffies();

            for(int i = 0; i < N; ++i) {
                int min_idx = i;
                for(int j = i + 1; j < N; ++j) {
                    if(data[j] < data[min_idx]) {
                        min_idx = j;
                    }
                }
                if(min_idx != i) {
                    int tmp = data[i];
                    data[i] = data[min_idx];
                    data[min_idx] = tmp;
                }
            }
            auto stop_time = isix::get_ujiffies();
            times_us[m] = stop_time - start_time;
            dbprintf("Selection sort complete!\n");
            dbprintf("Sorting time %d: %d us\n", m+1, (int)times_us[m]);
        }

        for(int k = 0; k < 3; ++k) {
            periph::gpio::set(led_0, 1);
            isix::wait_ms(500);

            periph::gpio::set(led_0, 0);
            isix::wait_ms(500);
        }
        float sum = 0;
        for(int m = 0; m < measurements; ++m)
            sum += times_us[m];
        float mean = sum / measurements;

        float sq_sum = 0;
        for(int m = 0; m < measurements; m++) {
            sq_sum += (times_us[m] - mean) * (times_us[m] - mean);
        }
        float std_dev = std::sqrt(sq_sum / (measurements - 1));
        int mean_int = static_cast<int>(mean);
        int mean_frac = static_cast<int>((mean - mean_int) * 1000);

        int std_int = static_cast<int>(std_dev);
        int std_frac = static_cast<int>((std_dev - std_int) * 1000);

        dbprintf("Average sorting time: %d.%d us\n", mean_int, mean_frac);
        dbprintf("Std deviation: %d.%d us\n", std_int, std_frac);
    }
}


auto main() -> int
{
	static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );
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
		periph::drivers::uart_early::open,
		"serial0", 115200
	);
    // Configure PD13 pin LED as an output
    periph::gpio::setup( led_0,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );
	isix::task_create( app::selection_sort, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
	isix::start_scheduler();
	return 0;
}
