int add(int a, int b)
{
    return a + b;
}

void start()
{
    int a = 0;
    for (int i = 0; i < 114514; i++) {
        a = add(a, 1);
        // move a to s7
        asm("mv s7, %0" : : "r"(a));

        // wait sometime
        for (int j = 0; j < 114514; j++);
    }

    for (;;);
}