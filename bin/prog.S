  .org 0x0
  .global __start
__start:
  xor a0, a0, a0
  li sp, 2048
  call start