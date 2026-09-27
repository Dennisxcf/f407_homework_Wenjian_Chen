// in taskbooster.cpp
#include "taskbooster.hpp"

#include "main.h"
#include "tx_api.h"

extern TX_THREAD led_thread;
extern uint8_t led_thread_stack[1024];
extern void led_thread_entry(ULONG thread_input);

#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void taskbooster(void)
{
    // Implement the task booster functionality here  
    tx_thread_create(&led_thread, TX_NAME("LED Thread"), led_thread_entry, 0x1234,
                     led_thread_stack, sizeof(led_thread_stack),
                     10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);
}
