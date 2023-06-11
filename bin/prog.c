#define LED_ADDR 0x400004

void out_led(short v) {
    *(unsigned int *)LED_ADDR = v;
}

int add(int a, int b) {
    return a + b;
}

void start()
{
    for (int i = 0; i < 114514; i++)
        out_led(i);

    for (;;);
}