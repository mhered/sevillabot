#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
private:
    int pinCLK;
    int pinDT;
    int pinSW;

    int previousCLK;
    int previousDT;
    int previousSW;

    long lastDebounceTime;
    int debounceDelay;

public:
    static const int NONE = 999;
    static const int CLOCKWISE = -1;
    static const int ANTICLOCKWISE = 1;
    static const int CLICK = 0;

    Encoder(int clk, int dt, int sw);

    int getCommand();
};

#endif // ENCODER_H

