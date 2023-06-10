  .org 0x0
  .global __start
__start:
  li a0, 30
  li a1, 60
  add a0, a0, a1

loop:
  add  a0, a0, a0
  j __start