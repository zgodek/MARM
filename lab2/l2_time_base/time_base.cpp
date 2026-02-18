#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <stm32_ll_tim.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <isix.h>


namespace app {
    void test_thread(void*) {
        for(;;) {}
    }
}

namespace {
	volatile uint8_t step = 0;
}

auto timer_config() -> void {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);

	uint32_t timer_clock = rcc_clocks.PCLK2_Frequency;
	
	if (LL_RCC_GetAPB2Prescaler() != LL_RCC_APB2_DIV_1) {
		timer_clock *= 2;
	}

	const uint32_t target_freq = 10000;
	uint32_t prescaler_value = (timer_clock / target_freq) - 1;

	LL_TIM_SetPrescaler(TIM1, prescaler_value);
	LL_TIM_SetAutoReload(TIM1, 2000 - 1);

	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableIT_UPDATE(TIM1);
	LL_TIM_GenerateEvent_UPDATE(TIM1);
}

auto output_config() -> void {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
}

extern "C" {
	void tim1_up_tim10_isr_vector() {
		if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
			LL_TIM_ClearFlag_UPDATE(TIM1);
			LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
			switch(step) {
				case 0:
					LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12);
					break;
				case 1:
					LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_13);
					break;
				case 2:
					LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_14);
					break;
				case 3:
					LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);
					break;
			}
			step++;
			if (step > 3) {
				step = 0;
			}
		}
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
	output_config();
	timer_config();
	isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< ISIX sample template >>>>");
	isix::start_scheduler();
	return 0;
}
