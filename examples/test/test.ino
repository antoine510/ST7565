#include "ST7565.h"

ST7565 screen;

void setup() {
    screen.begin(0x0f);
}

void loop() {
    screen.clear();
    screen.drawstring_aligned(0, 0, "TEST");
    screen.display();
    delay(1000);
}
