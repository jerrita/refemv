  .org 0x0
  .global __start
__start:
  xor a0, a0, a0
  li a1, 30
  li a2, 120
cmp:
  add a0, a0, a1
  blt a0, a2, cmp
loop:
  j loop