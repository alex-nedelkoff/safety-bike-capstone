#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"


// LED Strip Setup
#define LED_PIN_1 18 // GPIO Data Pin
#define LED_PIN_2 19 // GPIO Data Pin
#define NUM_LEDS 4 // num of LEDS per strip
#define BLINK_DURATION 5000 // 5 seconds blink duration
#define BRIGHTNESS 100 // brightness


Adafruit_NeoPixel strip1(NUM_LEDS, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(NUM_LEDS, LED_PIN_2, NEO_GRB + NEO_KHZ800);


// Audio Setup
HardwareSerial mySerial(2);
DFRobotDFPlayerMini player;


// ESP-NOW Data Structure
typedef struct struct_message {
    int command;
} struct_message;


struct_message message;


void blinkLED(Adafruit_NeoPixel &strip, uint32_t color) {
    unsigned long startTime = millis();
    while (millis() - startTime < BLINK_DURATION) {
        for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, color);
        }
        strip.show();
        delay(500);


        for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, 0);
        }
        strip.show();
        delay(500);
    }
}


void playAudio() {
    player.play(1); // Play audio file 1
}


void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&message, incomingData, sizeof(message));
    Serial.print("Received command: ");
    Serial.println(message.command);


    uint32_t dimmedColor = strip1.Color(BRIGHTNESS, 0, 0);


    if (message.command == 1) {
        playAudio();
        blinkLED(strip1, dimmedColor);
    }
    if (message.command == 2) {
        playAudio();
        blinkLED(strip2, dimmedColor);
    }
}


void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);


    // LED Setup
    strip1.begin();
    strip2.begin();
    strip1.setBrightness(BRIGHTNESS);
    strip2.setBrightness(BRIGHTNESS);
    strip1.show();
    strip2.show();


    // Audio Setup
    mySerial.begin(9600, SERIAL_8N1, 26, 27);
    if (player.begin(mySerial)) {
        Serial.println("DFPlayer Mini connected!");
        player.volume(20);
    } else {
        Serial.println("Failed to connect to DFPlayer Mini!");
    }


    // ESP-NOW Setup
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }


    esp_now_register_recv_cb(onDataReceive);
}


void loop() {
}
