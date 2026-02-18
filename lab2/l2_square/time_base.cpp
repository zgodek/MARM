#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>


namespace app {
    void test_thread(void*) {
        for(;;) {}
    }
}

auto output_config() -> void {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);

	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_12, LL_GPIO_AF_2);
	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_13, LL_GPIO_AF_2);
	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_14, LL_GPIO_AF_2);

	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
}

auto timer_config() -> void {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);

	uint32_t timer_clock = rcc_clocks.PCLK1_Frequency;
	
	if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB2_DIV_1) {
		timer_clock *= 2;
	}

	const uint32_t target_freq = 2000;
	uint32_t prescaler_value = (timer_clock / target_freq) - 1;

	LL_TIM_SetPrescaler(TIM4, prescaler_value);
	LL_TIM_SetAutoReload(TIM4, 1000 - 1);

	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_TOGGLE);

	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);

	LL_TIM_OC_SetCompareCH1(TIM4, 0);
	LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetCompareCH2(TIM4, 0);
	LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_LOW);
	LL_TIM_OC_SetCompareCH3(TIM4, 500);
	LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_LOW);

	LL_TIM_EnableCounter(TIM4);
	LL_TIM_GenerateEvent_UPDATE(TIM4);
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
