#pragma once

#define IO_SHIFB(i)    (1 << (2 + i))
#define IO_START       0x400000
#define IO_LED         (IO_START + IO_SHIFB(0))
#define IO_UART_DAT    (IO_START + IO_SHIFB(1))
#define IO_UART_CTR    (IO_START + IO_SHIFB(2))
#define IO_SW          (IO_START + IO_SHIFB(3))
#define IO_TIMER       (IO_START + IO_SHIFB(4))

#define TIMER_LEN      83  // 83.333ns


#define SetLED(x)      (*(volatile unsigned int *)IO_LED = (x))
#define GetSW()        (*(volatile unsigned int *)IO_SW)
#define GetTime()     (*(volatile unsigned int *)IO_TIMER)

#define HALT()         for (;;)