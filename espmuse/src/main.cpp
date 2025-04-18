#include "Arduino.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "freertos/queue.h"
#include <memory>

#include "Audio.h"
#include "museWrover.h"

#include "buttons.h"
#include "player.h"
#include "utils.h"

MuseLuxe ml;
ES8388 es;
Audio audio;
Player *player = nullptr;

const uint seek_step_sec = 15;
const uint8_t volume_step = 4;
uint8_t volume_current = 50;
uint8_t volume_max = 100;
int8_t last_encoder = 0;
int8_t last_direction = 0;
bool vol_button_held = false;
bool play_button_held = false;


struct I2CPacket {
    uint8_t buttons;
    int8_t encoder;
    int8_t direction;
};
I2CPacket received_packet;

void receivePacket() {
    Wire.readBytes((uint8_t *)&received_packet, sizeof(received_packet));

    bool btn_prev = !(received_packet.buttons & (1 << 0));
    bool btn_play = !(received_packet.buttons & (1 << 1));
    bool btn_next = !(received_packet.buttons & (1 << 2));
    bool btn_volume = !(received_packet.buttons & (1 << 3));

    handleI2CButton(btn_play, BTN_PLAY_I2C);
    handleI2CButton(btn_next, BTN_NEXT_I2C);
    handleI2CButton(btn_prev, BTN_PREV_I2C);
    handleI2CButton(btn_volume, BTN_VOL_I2C);

    if (received_packet.encoder != last_encoder) {
        if (received_packet.direction > 0) {
            if (volume_current <= (volume_max - volume_step)) 
                volume_current += volume_step;
            else
                volume_current = volume_max;
        } else if (received_packet.direction < 0) {
            if (volume_current >= volume_step)
                volume_current -= volume_step;
            else
                volume_current = 0;
        }

        log_i("Setting volume: %d", volume_current);
        es.volume(ES8388::ES_MAIN, volume_current);
        es.volume(ES8388::ES_OUT1, volume_current);
    }

    last_encoder = received_packet.encoder;
    last_direction = received_packet.direction;
}

void i2cTask(void *pv_params) {
    while (true) {
        Wire.requestFrom(0x20, sizeof(I2CPacket));
        if (Wire.available() == sizeof(received_packet)) {
            receivePacket();
        }
    }
}

void savePositionTask(void *pv_params) {
    const TickType_t freq = 10000 / portTICK_PERIOD_MS; // 10s
    TickType_t last = xTaskGetTickCount();
  
    while (true) {
        if (player && player->current_book && player->audio->isRunning()) {
            player->current_book->save();
        }
        vTaskDelayUntil(&last, freq);
    }
}

void buttonTask(void *pv_params) {
    ButtonEvent event;

    while (true) {
        if (xQueueReceive(button_queue, &event, portMAX_DELAY)) {
            log_i("Button: %d %d", event.button, event.type);

            switch (event.button) {
            case BTN_PLAY_I2C:
                if (!player || !player->current_book)
                    break;

                if (event.type == PRESS) {
                    audio.pauseResume();
                } else if (event.type == HOLD) {
                    play_button_held = true;
                } else if (event.type == RELEASE) {
                    play_button_held = false;
                }

                break;
            case BTN_NEXT_I2C:
                if (!player || !player->current_book)
                    break;

                if (play_button_held && event.type == PRESS) {
                    player->loadNext();
                    break;
                }

                if (vol_button_held && event.type == PRESS) {
                    player->current_book->playNext();
                    break;
                }

                if (event.type == PRESS || event.type == HOLD) {
                    player->current_book->seek(seek_step_sec);
                    break;
                }

                break;
            case BTN_PREV_I2C:
                if (!player || !player->current_book)
                    break;

                if (play_button_held && event.type == PRESS) {
                    player->loadPrev();
                    break;
                }

                if (vol_button_held && event.type == PRESS) {
                    player->current_book->playPrev();
                    break;
                }

                if (event.type == PRESS || event.type == HOLD) {
                    player->current_book->seek(-seek_step_sec-2);
                    break;
                }

                break;
            case BTN_PLAY_BUILTIN:
                if (player && player->current_book) {
                    audio.pauseResume();
                }
                break;
            case BTN_PLUS_BUILTIN:
                if (vol_button_held) {
                    player->loadNext();
                    break;
                }

                if (player && player->current_book) {
                    player->current_book->playNext();
                    break;
                }

                break;
            case BTN_MIN_BUILTIN:
                if (vol_button_held) {
                    player->loadPrev();
                    break;
                }

                if (player && player->current_book) {
                    player->current_book->playPrev();
                    break;
                }

                break;
            case BTN_VOL_I2C:
                if (event.type == HOLD) {
                    vol_button_held = true;
                } else if (event.type == RELEASE) {
                    vol_button_held = false;
                }
                break;
            }
        }
    }
}

void setup() {
    delay(1000);

    Serial.begin(115200);
    Serial.println("\r\nReset");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SPI.setFrequency(1000000);

    ml.begin();

    // Mount SD card.
    bool sd_ok = false;
    for (uint8_t i = 1; i <= 3; i++) {
        log_i("Mounting SD card (attempt %d/3) ...", i);

        if (sd_ok = SD.begin(SD_CS)) {
            break;
        }
        delay(500);
    }
    if (!sd_ok) {
        log_e("SD card init failed!");
        while (1)
            for (;;)
                ;
    }

    log_i("SD card total: %10u bytes", SD.totalBytes());
    log_i("SD card used:  %10u bytes", SD.usedBytes());

    if (!SPIFFS.begin()) {
        log_e("SPIFFS initialisation failed!");
        while (1)
            for (;;)
                ;
    }

    log_i("Connecting to ES8388 codec ... ");
    while (not es.begin(IIC_DATA, IIC_CLK)) {
        log_e("failed!");
        delay(1000);
    }
    log_i("ok");

    es.volume(ES8388::ES_MAIN, volume_current);
    es.volume(ES8388::ES_OUT1, volume_current);
    es.mute(ES8388::ES_OUT1, false);
    es.mute(ES8388::ES_MAIN, false);

    // Enable amplifier
    pinMode(GPIO_PA_EN, OUTPUT);
    digitalWrite(GPIO_PA_EN, HIGH);

    audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_MCLK);
    audio.setVolume(volume_current);

    button_queue = xQueueCreate(10, sizeof(ButtonEvent));

    // Handle button events
    pinMode(BUTTON_PAUSE, INPUT_PULLUP);
    pinMode(BUTTON_VOL_MINUS, INPUT_PULLUP);
    pinMode(BUTTON_VOL_PLUS, INPUT_PULLUP);
    attachInterruptArg(BUTTON_PAUSE, buttonISR, (void *)BTN_PLAY_BUILTIN, FALLING);
    attachInterruptArg(BUTTON_VOL_MINUS, buttonISR, (void *)BTN_MIN_BUILTIN, FALLING);
    attachInterruptArg(BUTTON_VOL_PLUS, buttonISR, (void *)BTN_PLUS_BUILTIN, FALLING);

    xTaskCreate(buttonTask, "buttonTask", 8192, NULL, 1, NULL);
    xTaskCreate(i2cTask, "i2cTask", 8192, NULL, 1, NULL);
    xTaskCreate(savePositionTask, "savePositionTask", 4096, NULL, 2, NULL);

    player = new Player(audio);
    if (!player->initialized)
        log_e("Failed to initialize player!");
    log_i("Player initilized");

    if (!player->resume()) {
        log_i("No saved book found. Loading first book ...");
        player->loadBook(0, true);
    }

    if (!player->current_book->resume()) {
        log_i("No saved position found. Loading first chapter ...");
        player->current_book->playChapter(0);
    }
}

void loop() {
    audio.loop();
    vTaskDelay(1);
}

void audio_info(const char *info) {
    Serial.print("info        ");
    Serial.println(info);
}

void audio_eof_mp3(const char *info) {
    log_i("End of chapter %d", player->current_book->idx_current);
    player->current_book->playNext();
    Serial.println(info);
}