#include "Arduino.h"
#include "Wire.h"

#include "RotaryEncoder.h"

const int SLAVE_ADDR = 0x20;
const unsigned long DEBOUNCE_DELAY = 50;
const int ENC_CLK_PIN = 14;
const int ENC_DT_PIN = 15;

struct I2CPacket {
    uint8_t buttons;
    int8_t encoder;
    int8_t direction;
};

struct Button {
    uint8_t pin;
    uint8_t state;
    uint8_t last_state;
    unsigned long time_debounce;
};

Button BTN_1 = {5, HIGH, HIGH, 0};
Button BTN_2 = {2, HIGH, HIGH, 0};
Button BTN_3 = {3, HIGH, HIGH, 0};
Button BTN_R = {4, HIGH, HIGH, 0};

int8_t encoder_pos = 0;
int8_t encoder_dir = 1;
RotaryEncoder encoder(ENC_CLK_PIN, ENC_DT_PIN, RotaryEncoder::LatchMode::TWO03);

bool checkButton(Button &button) {
    bool pressed = false;
    uint8_t val = digitalRead(button.pin);

    if (val != button.last_state)
        button.time_debounce = millis();

    if ((millis() - button.time_debounce) > DEBOUNCE_DELAY) {
        if (val != button.state) {
            button.state = val;
            if (button.state == LOW)
                pressed = true;
        }
    }

    button.last_state = val;
    return pressed;
}

uint8_t packBools(bool b1, bool b2, bool b3, bool b4) {
    return (b1 << 0) | (b2 << 1) | (b3 << 2) | (b4 << 3);
}

void requestEvent() {
    I2CPacket packet {
        .buttons = packBools(BTN_1.state, BTN_2.state, BTN_3.state, BTN_R.state),
        .encoder = encoder_pos,
        .direction = encoder_dir,
    };

    Wire.write((uint8_t *)&packet, sizeof(packet));
}

void setup() {
    Serial.begin(9600);

    pinMode(BTN_1.pin, INPUT_PULLUP);
    pinMode(BTN_2.pin, INPUT_PULLUP);
    pinMode(BTN_3.pin, INPUT_PULLUP);
    pinMode(BTN_R.pin, INPUT_PULLUP);

    pinMode(CLK_PIN, INPUT_PULLUP);
    pinMode(DT_PIN, INPUT_PULLUP);

    pinMode(LED_BUILTIN, OUTPUT);
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);

    Serial.println("Setup done ...");
}

void loop() {
    checkButton(BTN_1);
    checkButton(BTN_2);
    checkButton(BTN_3);
    checkButton(BTN_R);

    // Serial.print("BTN_1: ");  Serial.print(BTN_1.state);
    // Serial.print(" BTN_2: "); Serial.print(BTN_2.state);
    // Serial.print(" BTN_3: "); Serial.print(BTN_3.state);
    // Serial.print(" BTN_R: "); Serial.println(BTN_R.state);

    encoder.tick();

    int8_t new_pos = encoder.getPosition();
    if (encoder_pos != new_pos) {
        encoder_dir = (int8_t)encoder.getDirection();
        encoder_pos = new_pos;

        // Serial.print("pos:");
        // Serial.print(encoder_pos);
        // Serial.print(" dir:");
        // Serial.println(encoder_dir);
    }
    delay(5);
}