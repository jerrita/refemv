void output(int v)
{
    // wait sometime
    for (int j = 0; j < 114514; j++)
        ;
    __asm__("mv s7, %0" ::"r"(v));
}

void start()
{
    int i = 0;
    while (1)
    {
        output(i);
        i++;
    }
}