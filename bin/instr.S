  .org 0x0
  .global __start
__start:
  nop
  xor a0, a0, a0
  // Copy program (1MB) from flash to SPRAM
  // flash: 0x80000000, spram: 0x90000000
