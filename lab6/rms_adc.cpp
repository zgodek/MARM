#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <stm32f4xx_ll_adc.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_tim.h>
#include <cmath>
#include <vector>


static const bool GENERATE_SINE = false; 

const float PWM_PERIOD = 1000;
const float PWM_CENTER = PWM_PERIOD / 2.0f;
static const int WAVE_SAMPLES = 100;
static const int WAVE_FREQ_DELAY = 10;   // 10us delay = 1kHz przy 100 próbkach
static uint16_t wave_table[WAVE_SAMPLES];

static const int ADC_BUF_SIZE = 1000;
volatile uint16_t adc_buffer[ADC_BUF_SIZE];
volatile int adc_buf_index = 0;
static isix::semaphore adc_ready_sem(0, 1);

static const int led_thresholds[3] = {200, 400, 600};

volatile int target_vpp_mv = 3300; 

extern "C" {
    void adc_isr_vector(void);
}

namespace app {

    // Funkcja generująca SINUSA
    void update_sine_table(float vpp_mv) {
        if (vpp_mv > 3300.0f) vpp_mv = 3300.0f;
        if (vpp_mv < 0.0f) vpp_mv = 0.0f;
        float vpp_ratio = vpp_mv / 3300.0f;

        float pwm_amp = PWM_CENTER * vpp_ratio;

        for (int i = 0; i < WAVE_SAMPLES; i++) {
            // Kąt w radianach
            float angle = (2.0f * 3.14159f * i) / WAVE_SAMPLES;
            
            // Przeskalowanie do zakresu PWM
            wave_table[i] = (uint32_t)(PWM_CENTER + (sinf(angle) * pwm_amp));
        }
    }

    // Funkcja generująca TRÓJKĄT
    void update_triangle_table(float vpp_mv) {
        if (vpp_mv > 3300.0f) vpp_mv = 3300.0f;
        if (vpp_mv < 0.0f) vpp_mv = 0.0f;
        float vpp_ratio = vpp_mv / 3300.0f;

        float pwm_amp = PWM_CENTER * vpp_ratio;

        for (int i = 0; i < WAVE_SAMPLES; i++) {
            float sample_val = 0.0f;

            // Pierwsza połowa (wznoszenie od -1 do 1)
            if (i < WAVE_SAMPLES / 2) {
                float progress = (float)i / (WAVE_SAMPLES / 2.0f); 
                sample_val = -1.0f + (progress * 2.0f);
            } 
            // Druga połowa (opadanie od 1 do -1)
            else {
                float progress = (float)(i - WAVE_SAMPLES / 2) / (WAVE_SAMPLES / 2.0f);
                sample_val = 1.0f - (progress * 2.0f);
            }

            // Przeskalowanie do zakresu PWM
            wave_table[i] = (uint32_t)(PWM_CENTER + (sample_val * pwm_amp));
        }
    }

    void setup_pwm_generator() {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
        LL_GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = LL_GPIO_PIN_6;
        gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
        gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_init.Pull = LL_GPIO_PULL_NO;
        gpio_init.Alternate = LL_GPIO_AF_2;
        LL_GPIO_Init(GPIOA, &gpio_init);
        LL_TIM_SetPrescaler(TIM3, 0);
        LL_TIM_SetAutoReload(TIM3, PWM_PERIOD);
        LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_SetCompareCH1(TIM3, 0);
        LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
        LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
        LL_TIM_EnableCounter(TIM3);
        LL_TIM_EnableAllOutputs(TIM3);
    }

    void setup_adc_reader() {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
        LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
        LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
        LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
        ADC1->CR2 &= ~ADC_CR2_EXTEN;  
        ADC1->CR2 |= ADC_CR2_EXTEN_0; 
        LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
        LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
        LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_15CYCLES);
        LL_ADC_EnableIT_EOCS(ADC1);
        NVIC_SetPriority(ADC_IRQn, 10);
        NVIC_EnableIRQ(ADC_IRQn);
        LL_ADC_Enable(ADC1);
        LL_TIM_SetPrescaler(TIM2, 0);
        LL_TIM_SetAutoReload(TIM2, 2082); 
        LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
        LL_TIM_EnableCounter(TIM2);
    }

    void setup_leds() {
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

        LL_GPIO_InitTypeDef gpio_init = {};
        gpio_init.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
        gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
        gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
        gpio_init.Pull = LL_GPIO_PULL_NO;
        LL_GPIO_Init(GPIOD, &gpio_init);
    }

    // --- ZADANIE 1: Generator ---
    void generate_wave_task(void*) {
        if(GENERATE_SINE) update_sine_table(target_vpp_mv);
        else update_triangle_table(target_vpp_mv);

        setup_pwm_generator();
        
        int idx = 0;
        for(;;) {
            LL_TIM_OC_SetCompareCH1(TIM3, wave_table[idx]);
            idx++;
            if(idx >= WAVE_SAMPLES) idx = 0;
            
            isix::wait_us(WAVE_FREQ_DELAY); 
        }
    }

    // --- ZADANIE 2: Sterownik Pomiarów ---
    void measure_task(void*) {
        setup_adc_reader();
        
        // Parametry zadania
        int current_vpp = 100;
        int step = 250;
        int max_vpp = 3000;
        int measurements_count = 0;

        // Ustawienie początkowe fali
        target_vpp_mv = current_vpp;
        if(GENERATE_SINE) update_sine_table(target_vpp_mv);
        else update_triangle_table(target_vpp_mv);

        dbprintf("=== START LABORATORIUM: %s ===", GENERATE_SINE ? "SINUS" : "TROJKAT");
        isix::wait_ms(1000);

        for(;;) {
            if(adc_ready_sem.wait(ISIX_TIME_INFINITE) == ISIX_EOK) {
                
                long long sum_squares = 0;
                for(int i=0; i<ADC_BUF_SIZE; i++) {
                    int32_t val_ac = (int32_t)adc_buffer[i] - 2048; 
                    sum_squares += (val_ac * val_ac);
                }
                float rms_raw = std::sqrt(sum_squares / (float)ADC_BUF_SIZE);
                float rms_mv = (rms_raw * 3300.0f) / 4096.0f;
                int rms = (int)rms_mv;
                if (rms < led_thresholds[0]) {
                    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12);
                    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
                }
                else if (rms < led_thresholds[1]) {
                    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_13);
                    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);
                }
                else if (rms < led_thresholds[2]) {
                    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_14);
                    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15);
                }
                else {
                    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_15);
                    LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14);
                }
                dbprintf("Vpp_Set: %d mV | Pomiar %d/3 | RMS: %d mV", 
                         target_vpp_mv, measurements_count + 1, rms);

                measurements_count++;

                if (measurements_count >= 3) {
                    measurements_count = 0;
                    current_vpp += step;

                    if (current_vpp > max_vpp) {
                        dbprintf("=== KONIEC POMIAROW ===");
                        isix::wait_ms(10000); 
                        current_vpp = 100;
                    }
                    isix::wait_ms(5000); 

                    target_vpp_mv = current_vpp;
                    
                    if(GENERATE_SINE) update_sine_table(target_vpp_mv);
                    else update_triangle_table(target_vpp_mv);
                    
                    dbprintf(">>> ZMIANA NAPIECA NA: %d mVpp <<<", target_vpp_mv);
                } else {
                    isix::wait_ms(500);
                }
            }
        }
    }
}

extern "C" {
    void adc_isr_vector(void) {
        if(LL_ADC_IsActiveFlag_EOCS(ADC1)) {
            LL_ADC_ClearFlag_EOCS(ADC1);
            uint16_t val = LL_ADC_REG_ReadConversionData12(ADC1);
            if(adc_buf_index < ADC_BUF_SIZE) {
                adc_buffer[adc_buf_index++] = val;
            } else {
                adc_buf_index = 0;
                adc_ready_sem.signal_isr();
            }
        }
    }
}

auto main() -> int
{
    static isix::semaphore m_ulock_sem { 1, 1 };
    isix::wait_ms( 500 );
    app::setup_leds();
    dblog_init_locked(
        [](int ch, void*) { return periph::drivers::uart_early::putc(ch); },
        nullptr,
        []() { m_ulock_sem.wait(ISIX_TIME_INFINITE); },
        []() { m_ulock_sem.signal(); },
        periph::drivers::uart_early::open, "serial0", 115200
    );
    LL_RCC_ClocksTypeDef rcc_clocks;
    LL_RCC_GetSystemClocksFreq(&rcc_clocks);

    isix::task_create( app::generate_wave_task, nullptr, 1536, isix::get_min_priority() );
    isix::task_create( app::measure_task, nullptr, 1536, isix::get_min_priority() );

    isix::start_scheduler();
    return 0;
}