#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/drivers/serial/uart_early.hpp>
#include <periph/gpio/gpio.hpp>
#include <isix.h>
#include <stm32_ll_usart.h>
#include <stm32_ll_gpio.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <isix/arch/irq.h>

// --- KONFIGURACJA SPRZĘTOWA ---
void usart1_config() {
    // 1. Zegary
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    // 2. Konfiguracja GPIO (PB6 -> TX, PB7 -> RX)
    // Upewnij się, że masz konwerter USB-UART podpięty pod te piny!
    
    // --- PB6 (TX) ---
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);

    // --- PB7 (RX) ---
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_7);

    // 3. Konfiguracja UART
    LL_USART_Disable(USART1);
    
    LL_USART_InitTypeDef USART_InitStruct;
    LL_USART_StructInit(&USART_InitStruct);

    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(USART1, &USART_InitStruct);

    // 4. Włącz USART
    LL_USART_Enable(USART1);
}

// Wysyłanie jednego znaku
void send_char_polling(char c) {
    while (!LL_USART_IsActiveFlag_TXE(USART1)) {
    }
    LL_USART_TransmitData8(USART1, c);
}

// Wysyłanie całego stringa (wykorzystuje powyższą funkcję)
void send_string_polling(const char* str) {
    while (*str) {
        send_char_polling(*str++);
    }
    while (!LL_USART_IsActiveFlag_TC(USART1)); 
}

// Odbieranie jednego znaku
char receive_char_polling() {
    while (!LL_USART_IsActiveFlag_RXNE(USART1)) {
    }
    return LL_USART_ReceiveData8(USART1);
}


// --- APLIKACJA ---

namespace app {
    void test_thread(void*) {
        isix::wait_ms(100); 
        
        // Komunikat powitalny
        send_string_polling("\r\nTerminal testowy USART1 (Polling).\r\n");
        send_string_polling("Pisz na klawiaturze\r\n");

        for(;;) {
            // 1. Odbierz znak (funkcja blokująca)
            char received = receive_char_polling();

            // 2. Odeślij znak z powrotem (Echo)
            send_char_polling(received);
            
            // 3. Specjalna obsługa entera
            if (received == '\r') {
                send_char_polling('\n');
            }
        }
    }
}

auto main() -> int
{
    usart1_config();
    
    isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    
    isix::start_scheduler();
    
    return 0;
}