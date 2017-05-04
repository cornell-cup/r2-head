#include <Servo.h>
#include "R2Protocol.h"

#define BUFFER_LEN 256
byte buffer[BUFFER_LEN];
byte index = 0;

Servo headMotor;

void setup() {
    Serial.begin(9600);
    pinMode(3, OUTPUT);
    headMotor.attach(3, 1000, 2000);
}

byte dir = 90;
unsigned long endTime = 0;

void loop() {
    byte available = Serial.available();
    if (available > 0) {
        if (available + index > BUFFER_LEN) {
            index = BUFFER_LEN - available;
        }
        Serial.readBytes(buffer + index, available);
        index += available;
        
        uint8_t data[16];
        struct R2ProtocolPacket params;
        params.data_len = 16;
        params.data = data;
        int32_t read = R2ProtocolDecode(buffer, index, &params);
        if (read >= 0) {
            char command = (char) params.data[0];
            Serial.println(command);
            if (command == 'L' && params.data_len >= 3) { // Turn counter clockwise
                dir = 45;
                endTime = millis() + params.data[1] * 256 + params.data[2];
                Serial.println(dir);
            }
            else if (command == 'R' && params.data_len >= 3) { // Turn clockwise
                dir = 135;
                endTime = millis() + params.data[1] * 256 + params.data[2];
                Serial.println(dir);
            }
            memmove(buffer, buffer + read, index - read);
            index -= read;
        }
    }
    
    if (dir != 90) {
        if (millis() > endTime) {
            dir = 90;
            headMotor.write(dir);
        }
        else {
            headMotor.write(dir);
        }
        delay(15);
    }
}
