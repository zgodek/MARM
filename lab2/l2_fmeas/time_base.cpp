#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <isix.h>


namespace {
	uint32_t measured_timer_clock = 0;
}


namespace app {
    void measure_frequency(void*) {
        for(;;) {
			uint32_t captured_value = LL_TIM_IC_GetCaptureCH1(TIM3);
            uint32_t frequency = 0;
            if (captured_value > 0) {
                frequency = measured_timer_clock / captured_value;
            }
            dbprintf("Pomiar: CCR1=%u | Freq=%u Hz", captured_value, frequency);
			isix::wait_ms(1000);
		}
    }
}

auto timer_measure_config() -> void {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_2);

	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);

	LL_RCC_ClocksTypeDef rcc_clocks;
    LL_RCC_GetSystemClocksFreq(&rcc_clocks);
    measured_timer_clock = rcc_clocks.PCLK1_Frequency;
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
        measured_timer_clock *= 2;
    }
	LL_TIM_SetPrescaler(TIM3, 0); 
    LL_TIM_SetAutoReload(TIM3, 0xFFFF);

	LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
	LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
	LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
	LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
	LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_TI1FP1);
    LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_RESET);

    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM3);
}

auto timer_generator_config() -> void {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_1);
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);

	uint32_t timer_clock = rcc_clocks.PCLK1_Frequency;
	
	if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
		timer_clock *= 2;
	}

	const uint32_t target_freq = 4 * 1000000; //10 kHz
	uint32_t prescaler_value = 0;
	uint32_t auto_reload = (timer_clock / target_freq) - 1;

	LL_TIM_SetPrescaler(TIM2, prescaler_value);
	LL_TIM_SetAutoReload(TIM2, auto_reload);

	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);

	LL_TIM_OC_SetCompareCH1(TIM2, (auto_reload + 1) / 2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);

	LL_TIM_GenerateEvent_UPDATE(TIM2);
	LL_TIM_ClearFlag_UPDATE(TIM2);

	LL_TIM_EnableCounter(TIM2);
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
	timer_generator_config();
	timer_measure_config();
	isix::task_create( app::measure_frequency, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< ISIX sample template >>>>");
	isix::start_scheduler();
	return 0;
}
