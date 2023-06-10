ENTRY(__start)

MEMORY {
    ram : ORIGIN = 0x00000000, LENGTH = 0x00010000
}

SECTIONS {
    .text : {
        *(.text)
    } > ram
    .data : {
        *(.data)
    } > ram
    .bss : {
        *(.bss)
    } > ram
}