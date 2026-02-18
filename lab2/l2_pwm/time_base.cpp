#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <stm32_ll_exti.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <isix.h>

namespace {
	volatile uint32_t counter = 0;
	volatile uint32_t debounce_ms = 15;
}

namespace app {
    void test_thread(void*) {
        for(;;) {}
    }
}

auto timer_config() -> void {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
	LL_RCC_ClocksTypeDef rcc_clocks;
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);

	uint32_t timer_clock = rcc_clocks.PCLK1_Frequency;
	
	if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
		timer_clock *= 2;
	}

	const uint32_t target_freq = 200*100; //200Hz
	uint32_t prescaler_value = (timer_clock / target_freq) - 1;

	LL_TIM_SetPrescaler(TIM4, prescaler_value);
	LL_TIM_SetAutoReload(TIM4, 100 - 1);

	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);


	LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH1(TIM4, 20);
	LL_TIM_OC_SetCompareCH2(TIM4, 40);
	LL_TIM_OC_SetCompareCH3(TIM4, 60);
	LL_TIM_OC_SetCompareCH4(TIM4, 0);

	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

	LL_TIM_GenerateEvent_UPDATE(TIM4);
	LL_TIM_ClearFlag_UPDATE(TIM4);

	LL_TIM_EnableCounter(TIM4);
}

auto debounce_timer_config() -> void {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_RCC_ClocksTypeDef rcc_clocks;
    LL_RCC_GetSystemClocksFreq(&rcc_clocks);

    uint32_t timer_clock = rcc_clocks.PCLK1_Frequency;
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1) {
        timer_clock *= 2;
    }

    uint32_t prescaler = (timer_clock / 10000) - 1;
    LL_TIM_SetPrescaler(TIM2, prescaler);
    LL_TIM_SetAutoReload(TIM2, debounce_ms * 10 - 1);

    LL_TIM_EnableIT_UPDATE(TIM2);
    
    NVIC_SetPriority(TIM2_IRQn, 6); 
    NVIC_EnableIRQ(TIM2_IRQn);
}


//General config
auto input_output_config() -> void {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_14, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);

	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_12, LL_GPIO_AF_2);
	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_13, LL_GPIO_AF_2);
	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_14, LL_GPIO_AF_2);
	LL_GPIO_SetAFPin_8_15(GPIOD, LL_GPIO_PIN_15, LL_GPIO_AF_2);

	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinSpeed (GPIOD, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
}

auto interrupt_config() -> void {
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_0, LL_GPIO_SPEED_FREQ_LOW);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

	NVIC_SetPriority(EXTI0_IRQn, 5);
	NVIC_EnableIRQ(EXTI0_IRQn);
}

extern "C" {
	//! Exti 0 vector
	void exti0_isr_vector() {
        if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
            LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

            LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);
			if (counter >= 100) {
				counter = 0;
			} else {
				counter += 10;
			}
			LL_TIM_OC_SetCompareCH4(TIM4, counter);
            LL_TIM_SetCounter(TIM2, 0);
            LL_TIM_EnableCounter(TIM2);
        }
    }

    void tim2_isr_vector() {
        if(LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
            LL_TIM_ClearFlag_UPDATE(TIM2);

            static uint8_t stable_low_cnt = 0;

            if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) {
                stable_low_cnt = 0;
            } 
            else {
                stable_low_cnt++;

                if (stable_low_cnt >= 3) {
                    LL_TIM_DisableCounter(TIM2);
                    stable_low_cnt = 0;

                    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
                    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
                }
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
	input_output_config();
	debounce_timer_config();
	interrupt_config();
	timer_config();
	isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    dbprintf("<<<< ISIX sample template >>>>");
	isix::start_scheduler();
	return 0;
}
