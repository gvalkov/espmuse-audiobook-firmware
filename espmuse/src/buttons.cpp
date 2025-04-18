#include "Arduino.h"
#include "freertos/queue.h"

QueueHandle_t button_queue;

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
    unsigned long time_held;
};

ButtonState btn_states[7] = {{false, false, false, 0, 0}};

void IRAM_ATTR buttonISR(void *arg) {
    ButtonEvent event = {
        .button = (Button)(uint32_t)arg,
        .type = PRESS,
    };
    xQueueSendFromISR(button_queue, &event, NULL);
}

void handleI2CButton(boolean current_state, Button btn) {
    unsigned long currentTime = millis();
    ButtonState *state = &btn_states[btn];

    if (current_state && !state->was_pressed) {
        // log_i("Pressed: %d", btn);
        state->pressed = true;
        state->time_pressed = currentTime;

        ButtonEvent event = {.button = btn, .type = PRESS};
        xQueueSendFromISR(button_queue, &event, NULL);

    } else if (!current_state && state->was_pressed) {
        // log_i("Released: %d", btn);
        state->pressed = false;
        state->held = false;

        ButtonEvent event = {.button = btn, .type = RELEASE};
        xQueueSendFromISR(button_queue, &event, NULL);
    }

    if (current_state && (currentTime - state->time_pressed > 800)) {
        // log_i("Held: %d", btn);
        state->held = true;
        state->time_pressed = currentTime;

        ButtonEvent event = {.button = btn, .type = HOLD};
        xQueueSendFromISR(button_queue, &event, NULL);
    }

    state->was_pressed = current_state;
    state->was_held = state->held;
}