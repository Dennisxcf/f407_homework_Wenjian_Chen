//in taskled.cpp
#include "main.h"
#include "tx_api.h"

TX_THREAD led_thread;
uint8_t led_thread_stack[1024]={0};

[[noreturn]] void led_thread_entry(ULONG thread_input){

    UNUSED(thread_input);

    while (1) {
        // Implement the LED control functionality here
        HAL_GPIO_TogglePin(GPIOH , GPIO_PIN_10);
        tx_thread_sleep(500);
    }
}