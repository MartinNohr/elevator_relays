// elevator relay controls
// receives commands from remote
#include <SPI.h>
#include "RF24.h"

RF24 radio(D4, D8);

//struct package
//{
//    int id = 0;
//    bool up = 0;
//};
//
//typedef struct package Package;
//Package data;
// these must be the same in the relay code
enum { TXADR = 0, RXADR };
uint8_t addresses[][6] = { "Node1","Node0" };

const int ACTIVITY_LED = D0;
const int upRelay = 10;   // SD3 GPIO10
const int downRelay = D3;  // couldn't use SD2 GPIO9 on newer NODEMCU's, it crashes
// the last time the LED was on
unsigned long lastLedTime = 0;
// last time the elevator position was updated
unsigned long lastElevatorUpdate = 0;
// remember the last time we got a signal from a button
unsigned long lastSignal = 0;
// the control byte bits
#define UP 0
#define DOWN 1
// elevator position, infer from voltage inputs
byte elevatorPosition = 0;
byte newPosition = 0;
const int elevatorUP = D1;
const int elevatorDOWN = D2;

// blink the light
void blinkLED(int times, int timeBetween, int timeAfter) {
    while (times--) {
        digitalWrite(ACTIVITY_LED, LOW);
        delay(timeBetween);
        digitalWrite(ACTIVITY_LED, HIGH);
        if (times && timeAfter)
            delay(timeAfter);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println();
    Serial.println("init...");
    radio.begin();
    //  radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(15, 5);
    radio.setChannel(100);
    radio.setDataRate(RF24_1MBPS);
    //  radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, addresses[RXADR]);
    radio.startListening();
    digitalWrite(upRelay, HIGH);
    pinMode(upRelay, OUTPUT);
    digitalWrite(downRelay, HIGH);
    pinMode(downRelay, OUTPUT);
    pinMode(ACTIVITY_LED, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
    pinMode(elevatorUP, INPUT_PULLUP);
    pinMode(elevatorDOWN, INPUT_PULLUP);
    Serial.println("listening");
    blinkLED(1, 500, 0);
    lastLedTime = millis();
}


void loop()
{
    // check if we should flash the LED to signal we're alive
    if (lastLedTime + 5000 < millis()) {
        //    bool light = digitalRead(ACTIVITY_LED);
        blinkLED(1, 5, 0);
        lastLedTime = millis();
    }
    byte data = 0;
    if (lastSignal == 0)
        lastSignal = millis();
    if (radio.available())
    {
        blinkLED(1, 50, 0);
        while (radio.available())
        {
            radio.read(&data, sizeof(data));
            // remember when we got a message
            lastSignal = millis();
        }
        if (bitRead(data, UP))
            Serial.println("UP");
        if (bitRead(data, DOWN))
            Serial.println("DOWN");
        if (data == 0)
            Serial.println("OFF");
        // turn on/off relays as specified
        digitalWrite(downRelay, !bitRead(data, DOWN));
        digitalWrite(upRelay, !bitRead(data, UP));
    }
    // see if we need to turn things off
    if (millis() > lastSignal + 5000) {
        digitalWrite(upRelay, HIGH);
        digitalWrite(downRelay, HIGH);
    }
    // check the elevator limit switch inputs and set the elevator positions
    // first read the new positions from the hardware
    newPosition = 0;
    if (digitalRead(elevatorUP) == 0)
        bitSet(newPosition, UP);
    if (digitalRead(elevatorDOWN) == 0)
        bitSet(newPosition, DOWN);
    if (newPosition != elevatorPosition || lastElevatorUpdate + 10000 < millis()) {
        lastElevatorUpdate = millis();
        elevatorPosition = newPosition;
        Serial.println("sending position change");
        // transmit the change
        radio.stopListening();
        radio.openWritingPipe(addresses[TXADR]);
        if (!radio.write(&elevatorPosition, sizeof elevatorPosition, true)) {
            Serial.println("xmit elevator position failed");
        }
        radio.startListening();
    }
    delay(10);
}
