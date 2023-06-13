#include "../inc/refem.h"

void prog() {
    SetLED(1.4 * 3);
}

void start() {
  int tm_s = GetTime();
  prog();
  int tm_e = GetTime();
  int tm_d = tm_e - tm_s;
  SetLED(tm_d - 52);  // 52 is the baseline for empty prog
  HALT();
}