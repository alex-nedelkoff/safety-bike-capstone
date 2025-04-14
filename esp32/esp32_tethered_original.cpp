#include <esp_now.h>
#include <WiFi.h>


uint8_t receiverAddress[] = {0xE4, 0x65, 0xB8, 0x21, 0x33, 0x30};  // MAC address for LED esp32


#define BUTTON_1 4  // GPIO button for LED strip 1
#define BUTTON_2 5  // GPIO button for LED strip 2


typedef struct struct_message { // defining message structure
    int command;
} struct_message;


struct_message message;


void sendCommand(int cmd) { // sending commands function
    message.command = cmd;
    esp_now_send(receiverAddress, (uint8_t *)&message, sizeof(message));
}


void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA); // setting esp32 as station


    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);


    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }


    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;


    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}


void loop() {
    if (digitalRead(BUTTON_1) == LOW) {
        sendCommand(1); // initiate LED strip 1
        delay(500);
    }
    if (digitalRead(BUTTON_2) == LOW) {
        sendCommand(2); // initiate LED strip 2
        delay(500);
    }
}
