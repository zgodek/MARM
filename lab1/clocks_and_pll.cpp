/*
 * platform_setup.cpp
 *  Platform initializaton specific code
 *  Created on: 20 lis 2013
 *      Author: lucck
 */
#include <stm32_ll_rcc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_system.h>
#include <stm32_ll_gpio.h>
#include <config/conf.h>
#include <functional>
#include <isix/arch/irq.h>
#include <isix.h>
#include <boot/arch/arm/cortexm/irq_vectors_table.h>
#include <boot/arch/arm/cortexm/crashinfo.h>

namespace drv {
namespace board {
namespace {


/** STM32 F3 system startup
 * 72MHz for AHB, APB2 36MHz APB1
 */
bool uc_periph_setup()
{

	constexpr auto retries=100000;

	isix_set_irq_vectors_base( &_exceptions_vectors );

    //! Deinitialize RCC
    LL_RCC_DeInit();
	LL_FLASH_SetLatency( LL_FLASH_LATENCY_3 );
	LL_FLASH_EnablePrefetch();
	//! Set MCU Prescallers
	LL_RCC_SetAHBPrescaler( LL_RCC_SYSCLK_DIV_1 );
	LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_1 );
	LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
	//! Enable HSE generator
	LL_RCC_HSE_Enable();
	for( int i=0; i<retries; ++i ) {
		if(LL_RCC_HSE_IsReady()) {
			break;
		}
	}
	if( !LL_RCC_HSE_IsReady() ) {
		return false;
	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, 8, 400, 4);
	LL_RCC_PLL_Enable();
	//Enable clocks for GPIOS
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA|LL_AHB1_GRP1_PERIPH_GPIOB|
		LL_AHB1_GRP1_PERIPH_GPIOC|LL_AHB1_GRP1_PERIPH_GPIOD|LL_AHB1_GRP1_PERIPH_GPIOE );

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	for( auto r=0; r<retries; ++r ) {
		if( LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL ) {
			break;
		}
	}
	return  LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL;
}



//! Application crash called from hard fault
void application_crash( crash_mode type, unsigned long* sp )
{
#ifdef PDEBUG
	cortex_cm3_print_core_regs( type, sp );
#else
	static_cast<void>(type);
	static_cast<void>(sp);
#endif
	for(;;) asm volatile("wfi\n");
}


}	//Unnamed NS

extern "C" {


//! This function is called just before call global constructors
void _external_startup(void)
{
	//SysTimer values
	//Give a chance a JTAG to reset the CPU
	for(unsigned i=0; i<1000000; i++) asm volatile("nop\n");

	//Initialize system perhipheral

	if( uc_periph_setup() ) {
		//1 bit for preemtion priority
		isix_set_irq_priority_group( isix_cortexm_group_pri7 );
		//Initialize isix
		isix::init(CONFIG_HCLK_HZ);
	} else {
		//TODO: Handle failure initialization
		//! Initialization failure
		for(;;);
	}
}


//Crash info interrupt handler
[[gnu::naked]]
void hard_fault_exception_vector(void)
{
	_cm3_hard_hault_entry_fn( application_crash );
}

#ifdef PDEBUG
int _write (int /*file*/, const void * /*ptr*/, size_t /*len*/)  { return -1; }
int _read (int /*file*/, void * /*ptr*/, size_t /*len*/)  { return -1; }
off_t _lseek (int /*file*/, off_t /*ptr*/, int /*dir*/)  { return -1; }
int _close (int /*file*/)  { return -1; }
#endif // PDEBUG

} /* extern C */

 
}}	//NS drv

