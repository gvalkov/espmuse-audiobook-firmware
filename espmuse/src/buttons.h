#pragma once

#include "Arduino.h"
#include "freertos/queue.h"

extern QueueHandle_t button_queue;

typedef enum Button : uint8_t {
    BTN_PLAY_BUILTIN,
    BTN_PLUS_BUILTIN,
    BTN_MIN_BUILTIN,
    BTN_PLAY_I2C,
    BTN_PREV_I2C,
    BTN_NEXT_I2C,
    BTN_VOL_I2C,
} Button;

typedef enum ButtonEventType : uint8_t {
    RELEASE = 0,
    PRESS = 1,
    HOLD = 2,
} ButtonEventType;

struct ButtonEvent {
    Button button;
    ButtonEventType type;
};

struct ButtonState {
    bool pressed;
    bool held;
    bool was_pressed;
    bool was_held;
    unsigned long time_pressed;
};

void IRAM_ATTR buttonISR(void *arg);
void handleI2CButton(boolean current_state, Button btn);
