#include "Encoder.h"
#include <Arduino.h> // This include is required for the Arduino-specific functions like pinMode, digitalRead, etc.

Encoder::Encoder(int clk, int dt, int sw) : pinCLK(clk), pinDT(dt), pinSW(sw), lastDebounceTime(0), debounceDelay(1) {
    pinMode(pinSW, INPUT_PULLUP);
    previousCLK = digitalRead(pinCLK);
    previousDT = digitalRead(pinDT);
    previousSW = digitalRead(pinSW);
}

int Encoder::getCommand() {
    int command = NONE;
    if ((millis() - lastDebounceTime) > debounceDelay) {
        int currentCLK = digitalRead(pinCLK);
        int currentDT = digitalRead(pinDT);
        int currentSW = digitalRead(pinSW);

        lastDebounceTime = millis();

        if ((previousCLK == HIGH) && (currentCLK == LOW)) {
            command = (currentDT == HIGH) ? CLOCKWISE : ANTICLOCKWISE;
        }

        if (previousSW == HIGH && currentSW == LOW) {
            command = CLICK;
        }

        previousCLK = currentCLK;
        previousDT = currentDT;
        previousSW = currentSW;
    }
    return command;
}

