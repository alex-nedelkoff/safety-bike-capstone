#include <esp_now.h>
#include <WiFi.h>

// MAC address of the receiver (wireless/helmet ESP32) - MAKE SURE THIS IS CORRECT
uint8_t receiverAddress[] = {0xE4, 0x65, 0xB8, 0x21, 0x33, 0x30}; // <-- Check this address

// Define button GPIO pins
#define BUTTON_1 4  // Button for LED strip 1
#define BUTTON_2 5  // Button for LED strip 2

// Define message structure to handle both commands and labels
#define MAX_LABEL_LENGTH 32 // Max characters for the object label
typedef struct struct_message {
    int command;  // Original command field
    char label[MAX_LABEL_LENGTH];  // New label field
    bool isLabel;  // Flag to indicate if this is a label message
} struct_message;

struct_message message;

// ESP-NOW Send Callback Function (Optional but good practice)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\\r\\nLast Packet Send Status:\\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendCommand(int cmd) {
    message.command = cmd;
    message.isLabel = false;  // Indicate this is a command message
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&message, sizeof(message));

    if (result == ESP_OK) {
        Serial.print("Command sent: ");
        Serial.println(cmd);
    } else {
        Serial.print("Error sending command: ");
        Serial.println(result);
    }
}

// Buffer for incoming serial data
char serialBuffer[MAX_LABEL_LENGTH + 1]; // +1 for null terminator
int serialBufferIndex = 0;

// --- Non-Blocking Debounce Variables --- 
const unsigned long DEBOUNCE_DELAY = 50; // milliseconds

// Button 1
int lastButton1State = HIGH;         // Previous RAW state from digitalRead
unsigned long lastButton1DebounceTime = 0; 
int button1State = HIGH;             // Confirmed STABLE state after debounce

// Button 2
int lastButton2State = HIGH;         // Previous RAW state from digitalRead
unsigned long lastButton2DebounceTime = 0; 
int button2State = HIGH;             // Confirmed STABLE state after debounce

void setup() {
    Serial.begin(115200); 
    Serial.println("--- Checkpoint 1: Serial Started ---"); 
    // Original messages moved after checkpoints for clarity
    // Serial.println("ESP32 Tethered Sender Starting...");
    // Serial.println("Waiting for object labels via Serial..."); 

    WiFi.mode(WIFI_STA); 
    Serial.println("--- Checkpoint 2: WiFi Mode Set ---"); 
    Serial.print("MAC Address (this device): "); 
    Serial.println(WiFi.macAddress());
    Serial.println("--- Checkpoint 3: MAC Address Printed ---"); 

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) { 
        Serial.println("!!! ESP-NOW Init Failed !!!"); 
        while(1); // Halt execution on failure
    }
    Serial.println("--- Checkpoint 4: ESP-NOW Initialized ---"); 

    // Register send callback
    esp_now_register_send_cb(OnDataSent); 
    Serial.println("--- Checkpoint 5: Send Callback Registered ---"); 

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0; 
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("!!! Failed to add peer !!!");
        while(1); // Halt execution on failure
    }
    Serial.println("--- Checkpoint 6: Peer Added ---"); 

    // --- Button setup ---
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
    Serial.println("--- Checkpoint 7a: Button PinModes Set ---");

    // Initialize serial buffer 
    memset(serialBuffer, 0, sizeof(serialBuffer));
    serialBufferIndex = 0;
    Serial.println("--- Checkpoint 7b: Serial Buffer Initialized ---"); 

    // --- Initialize button states ---
    lastButton1State = digitalRead(BUTTON_1);
    button1State = lastButton1State; // Initialize stable state too
    lastButton2State = digitalRead(BUTTON_2);
    button2State = lastButton2State; // Initialize stable state too
    Serial.println("--- Checkpoint 7c: Button States Initialized ---"); 
    
    Serial.println("--- Setup Complete --- Starting Loop ---");
}

void loop() {
    Serial.println("Loop Start"); 

    // --- Process ONE Serial Byte (if available) ---
    if (Serial.available() > 0) {
        char incomingByte = Serial.read();

        if (incomingByte == '\n') { // End of message detected
            if (serialBufferIndex > 0) { // We have a complete message
                serialBuffer[serialBufferIndex] = '\0'; // Null terminate
                // Serial.print("Received complete label from RPi: '"); // Keep prints minimal for now
                // Serial.print(serialBuffer);
                // Serial.println("'");
                message.isLabel = true; 
                strncpy(message.label, serialBuffer, MAX_LABEL_LENGTH);
                message.label[MAX_LABEL_LENGTH - 1] = '\0'; // Ensure null termination (strncpy safety)
                // Serial.print("Copied to ESP-NOW message.label: '"); 
                // Serial.print(message.label);
                // Serial.println("'");
                esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&message, sizeof(message));
                // if (result == ESP_OK) { Serial.println("Label sent via ESP-NOW."); }
                // else { Serial.print("Error sending label via ESP-NOW: "); Serial.println(result); }
            }
            // Reset buffer for next message regardless of whether we processed
            serialBufferIndex = 0;
            memset(serialBuffer, 0, sizeof(serialBuffer));

        } else if (incomingByte >= ' ' && incomingByte <= '~') { // Store printable characters
            if (serialBufferIndex < MAX_LABEL_LENGTH) { // Prevent buffer overflow
                serialBuffer[serialBufferIndex++] = incomingByte;
            } else {
                // Buffer overflow! Discard message and reset buffer
                // Serial.println("Serial buffer overflow, discarding message.");
                serialBufferIndex = 0;
                memset(serialBuffer, 0, sizeof(serialBuffer));
            }
        } else {
             // Ignore other characters like \r if present
        }
    } // End of single byte processing

    // --- Refined Non-Blocking Button Reading --- 
    unsigned long currentTime = millis();

    // Button 1 Check
    int reading1 = digitalRead(BUTTON_1); 
    Serial.print("B1 Raw Read: "); Serial.print(reading1); Serial.print(" | Stable: "); Serial.println(button1State); // Show raw vs stable

    if (reading1 != lastButton1State) {
        lastButton1DebounceTime = currentTime;
        Serial.println("B1 Changed! (Timer Reset)"); 
    }

    if ((currentTime - lastButton1DebounceTime) > DEBOUNCE_DELAY) {
        // If the reading has been stable for long enough...
        // check if the stable state is different from the last recorded stable state
        if (reading1 != button1State) {
            Serial.print("B1 Stable State Change Detected: "); Serial.println(reading1);
            button1State = reading1; // Update the confirmed stable state

            // If the new stable state is LOW (button pressed)
            if (button1State == LOW) {
                Serial.println("--> Button 1 Stable LOW (Pressed) - Sending Command");
                sendCommand(1);
            } else {
                Serial.println("--> Button 1 Stable HIGH (Released)"); 
            }
        }
    }
    lastButton1State = reading1; // Update the last raw reading for next time

    // Button 2 Check (Mirror logic)
    int reading2 = digitalRead(BUTTON_2); 
    Serial.print("B2 Raw Read: "); Serial.print(reading2); Serial.print(" | Stable: "); Serial.println(button2State);

    if (reading2 != lastButton2State) {
        lastButton2DebounceTime = currentTime;
        Serial.println("B2 Changed! (Timer Reset)");
    }
    if ((currentTime - lastButton2DebounceTime) > DEBOUNCE_DELAY) {
        if (reading2 != button2State) {
            Serial.print("B2 Stable State Change Detected: "); Serial.println(reading2);
            button2State = reading2; 
            if (button2State == LOW) {
                Serial.println("--> Button 2 Stable LOW (Pressed) - Sending Command");
                sendCommand(2);
            } else {
                Serial.println("--> Button 2 Stable HIGH (Released)");
            }
        }
    }
    lastButton2State = reading2; 

    Serial.println("Loop End"); 
    delay(100); // Increase delay slightly for readability
}