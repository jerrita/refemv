#define LED_ADDR 0x400004
#define UART_ADDR 0x400008
#define UART_CNTL 0x400010
#define SW_ADDR 0x400020

void wait()
{
    for (int i = 0; i < 100000; i++)
        ;
}

void out_led(char v)
{
    // wait();
    *(char *)LED_ADDR = v;
}

char get_sw()
{
    return *(char *)SW_ADDR;
}

int get_swi(int i)
{
    char val = get_sw();
    return val & (0b10000000 >> (i - 1));
}

int get_cntl()
{
    return *(int *)UART_CNTL;
}

void outc(char c)
{
    // while (get_cntl() & (1 << 9));
    *(char *)UART_ADDR = c;
}

void start()
{
    char s[] = "Hello, world!";
    int times = 20;
    while (times--)
    {
        for (int i = 0; i < 15; i++)
        {
            outc(s[i]);
        }
    }

    for (int i = 0; i < 100; i++)
        out_led(i);

    for (;;)
        ;
}