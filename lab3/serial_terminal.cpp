extern "C" {
    #include <sys/stat.h>

    // --- Missing functions (REQUIRED) ---
    int _fstat(int /*file*/, struct stat *st) {
        st->st_mode = S_IFCHR;
        return 0;
    }

    int _isatty(int /*file*/) { return 1; }

    void _exit(int /*status*/) { while(1); }

    int _kill(int /*pid*/, int /*sig*/) { return -1; }

    int _getpid(void) { return 1; }
}

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
#include <cstdio>
#include <cstring>

namespace {
    volatile char g_rx_char = 0;
    isix::semaphore g_rx_sem(0, 1);
    constexpr int TX_BUF_SIZE = 256; 
    volatile char g_tx_buffer[TX_BUF_SIZE];
    volatile int g_tx_head = 0;
    volatile int g_tx_tail = 0;
}

// --- KONFIGURACJA SPRZĘTOWA ---
void usart1_config() {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_7);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_7);

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

    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
    LL_USART_EnableIT_RXNE(USART1);

    LL_USART_Enable(USART1);
}

auto input_output_config() -> void {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD | LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOD, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);

	LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_14, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(GPIOD, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
}

void send_char_interrupts(char c) {
    g_tx_buffer[g_tx_head] = c;
    g_tx_head = (g_tx_head + 1) % TX_BUF_SIZE;
    LL_USART_EnableIT_TXE(USART1);
}

void send_string_interrupts(const char* str) {
    while (*str) {
        send_char_interrupts(*str++);
    }
}

char receive_char_interrupts() {
    int ret = g_rx_sem.wait(ISIX_TIME_INFINITE);
    
    if (ret != ISIX_EOK) {
        return 0;
    }

    return g_rx_char;
}

void control_led(int led_num, bool state) {
    uint32_t pin = 0;
    switch(led_num) {
        case 3: pin = LL_GPIO_PIN_13; break; // Orange
        case 4: pin = LL_GPIO_PIN_12; break; // Green
        case 5: pin = LL_GPIO_PIN_14; break; // Red
        case 6: pin = LL_GPIO_PIN_15; break; // Blue
        default: return;
    }
    
    if (state) LL_GPIO_SetOutputPin(GPIOD, pin);
    else       LL_GPIO_ResetOutputPin(GPIOD, pin);
}

bool get_button_state() {
    return LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
}

auto get_free_heap_bytes() -> std::size_t {
    isix::memory_stat mem_info;
    
    isix::heap_stats(mem_info);
    
    return mem_info.free;
}

auto get_cpu_usage() -> int {
    return isix::cpuload();
}

extern "C" {
    void usart1_isr_vector(void) {
        if (LL_USART_IsActiveFlag_RXNE(USART1)) {
            g_rx_char = LL_USART_ReceiveData8(USART1);
            g_rx_sem.signal_isr();
        }

        if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)) {
            if (g_tx_head != g_tx_tail) {
                LL_USART_TransmitData8(USART1, g_tx_buffer[g_tx_tail]);
                
                g_tx_tail = (g_tx_tail + 1) % TX_BUF_SIZE;
            } 
            else {
                LL_USART_DisableIT_TXE(USART1);
            }
        }
    }
}

// --- APLIKACJA ---

namespace app {
    static char cmd_buffer[64];
    static int cmd_idx = 0;

    void parse_command() {
        send_string_interrupts("\r\n");
        
        char reply[64];

        // 1. Komenda: help
        if (strcmp(cmd_buffer, "help") == 0) {
            send_string_interrupts("Dostepne komendy:\r\n");
            send_string_interrupts("led [3-6] on   - Wlacz diode\r\n");
            send_string_interrupts("led [3-6] off  - Wylacz diode\r\n");
            send_string_interrupts("btn            - Stan przycisku USER\r\n");
            send_string_interrupts("mem            - Wolna pamiec RAM\r\n");
            send_string_interrupts("cpu            - Uzycie procesora\r\n");
        }
        // 2. Komenda: btn (przycisk)
        else if (strcmp(cmd_buffer, "btn") == 0) {
            if (get_button_state()) send_string_interrupts("USER Button: WCISNIETY\r\n");
            else                    send_string_interrupts("USER Button: PUSZCZONY\r\n");
        }
        // 3. Komenda: mem (pamięć)
        else if (strcmp(cmd_buffer, "mem") == 0) {
            std::size_t free = get_free_heap_bytes();
            std::snprintf(reply, sizeof(reply), "Free Heap: %u bytes\r\n", (unsigned int)free);
            send_string_interrupts(reply);
        }
        // 4. Komenda: cpu (procesor)
        else if (strcmp(cmd_buffer, "cpu") == 0) {
            int load = get_cpu_usage();
            std::snprintf(reply, sizeof(reply), "CPU Load: %d.%d%%\r\n", load / 10, load % 10);
            send_string_interrupts(reply);
        }
        // 5. Komendy LED: "led 3 on", "led 5 off"
        else if (strncmp(cmd_buffer, "led", 3) == 0) {            
            int led_num = cmd_buffer[4] - '0'; // Konwersja char na int
            
            if (led_num >= 3 && led_num <= 6) {
                if (strstr(cmd_buffer, " on")) {
                    control_led(led_num, true);
                    send_string_interrupts("OK. LED ON.\r\n");
                } 
                else if (strstr(cmd_buffer, " off")) {
                    control_led(led_num, false);
                    send_string_interrupts("OK. LED OFF.\r\n");
                } else {
                    send_string_interrupts("Blad: uzyj 'on' lub 'off'\r\n");
                }
            } else {
                send_string_interrupts("Blad: zly numer diody (3-6)\r\n");
            }
        }
        else {
            if(strlen(cmd_buffer) > 0)
                send_string_interrupts("Nieznana komenda. Wpisz 'help'.\r\n");
        }

        send_string_interrupts("> ");
        
        cmd_idx = 0;
        memset(cmd_buffer, 0, sizeof(cmd_buffer));
    }

    void test_thread(void*) {
        isix::wait_ms(100); 
        send_string_interrupts("\r\n=== STM32 ISIX SHELL ===\r\n");
        send_string_interrupts("Wpisz 'help' aby zobaczyc liste komend.\r\n> ");

        memset(cmd_buffer, 0, sizeof(cmd_buffer));

        for(;;) {
            char c = receive_char_interrupts();

            if (c == '\r') {
                parse_command();
            }
            else if (c == 127 || c == '\b') {
                if (cmd_idx > 0) {
                    cmd_idx--;
                    cmd_buffer[cmd_idx] = 0;
                    send_string_interrupts("\b \b");
                }
            }
            else {
                if (cmd_idx < (int)sizeof(cmd_buffer) - 1) {
                    cmd_buffer[cmd_idx++] = c;
                    send_char_interrupts(c);
                }
            }
        }
    }
}

auto main() -> int
{
    usart1_config();
    input_output_config();
    
    isix::task_create( app::test_thread, nullptr, 1536, isix::get_min_priority() );
    
    isix::start_scheduler();
    
    return 0;
}