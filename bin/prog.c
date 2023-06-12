#define LED_ADDR 0x400004
#define UART_ADDR 0x400008
#define UART_CNTL 0x400010

void out_led(char v)
{
    // wait sometime
    // for (int i = 0; i < 100000; i++)
        ;
    *(char *)LED_ADDR = v;
}

void outc(char c)
{
    // while ((*(char *)UART_CNTL) & 0x200)
        ;
    *(char *)UART_ADDR = c;
}

void start()
{
    char s[] = "Hello, world!\n";
    int times = 20;
    while (times--)
    {
        out_led(times);
        for (int i = 0; i < 15; i++)
        {
            outc(s[i]);
        }
    }

    for (;;)
        ;
}