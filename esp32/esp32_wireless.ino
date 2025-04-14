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


// ESP-NOW Data Structure - Combined for both command and label
#define MAX_LABEL_LENGTH 32 // Max characters for the object label
typedef struct struct_message {
    int command;  // Original command field
    char label[MAX_LABEL_LENGTH];  // New label field
    bool isLabel;  // Flag to indicate if this is a label message
} struct_message;


struct_message message;


// Add at the top with other defines
#define AUDIO_DEBOUNCE_TIME 2000  // 2 seconds minimum between audio plays
unsigned long lastAudioPlayTime = 0;  // Track last time audio was played


// --- Add this global variable ---
char lastPlayedLabel[MAX_LABEL_LENGTH] = ""; // Store the last label played


// --- Add this global flag ---
volatile bool isAudioPlaying = false; // Track if we expect audio to be playing


// --- LED Functions ---
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

// --- Audio & Action Mapping ---
void playAudioForLabel(const char* label) {
    Serial.print("Processing label: ");
    Serial.println(label);

    // --- Check if player is currently supposed to be playing ---
    if (isAudioPlaying) {
        Serial.println("Skipping audio - Waiting for previous track to finish.");
        return;
    }

    // --- Check if same as last label ---
    if (strcmp(label, lastPlayedLabel) == 0) {
        Serial.println("Skipping audio - same as last played label");
        return;
    }

    // --- Check time debounce ---
    unsigned long currentTime = millis();
    if (currentTime - lastAudioPlayTime < AUDIO_DEBOUNCE_TIME) {
        Serial.println("Skipping audio - too soon since last play");
        return;
    }

    // --- If we get here, we will attempt to play ---
    bool playCommandSent = false; // Flag to track if play command was sent in this call

    // Map labels to audio tracks
    if (strcmp(label, "person") == 0) {
        Serial.println("Playing track 1 (person)");
        player.play(1);
        playCommandSent = true;
    }
    else if (strcmp(label, "bicycle") == 0) {
        Serial.println("Playing track 2 (bicycle)");
        player.play(2);
        playCommandSent = true;
    }
    else if (strcmp(label, "car") == 0) {
        Serial.println("Playing track 3 (car)");
        player.play(3);
        playCommandSent = true;
    }
    else if (strcmp(label, "motorcycle") == 0) {
        Serial.println("Playing track 4 (motorcycle)");
        player.play(4);
        playCommandSent = true;
    }
    else if (strcmp(label, "airplane") == 0) {
        Serial.println("Playing track 5 (airplane)");
        player.play(5);
        playCommandSent = true;
    }
    else if (strcmp(label, "bus") == 0) {
        Serial.println("Playing track 6 (bus)");
        player.play(6);
        playCommandSent = true;
    }
    else if (strcmp(label, "train") == 0) {
        Serial.println("Playing track 7 (train)");
        player.play(7);
        playCommandSent = true;
    }
    else if (strcmp(label, "truck") == 0) {
        Serial.println("Playing track 8 (truck)");
        player.play(8);
        playCommandSent = true;
    }
    else if (strcmp(label, "boat") == 0) {
        Serial.println("Playing track 9 (boat)");
        player.play(9);
        playCommandSent = true;
    }
    else if (strcmp(label, "traffic light") == 0) {
        Serial.println("Playing track 10 (traffic light)");
        player.play(10);
        playCommandSent = true;
    }
    else if (strcmp(label, "fire hydrant") == 0) {
        Serial.println("Playing track 11 (fire hydrant)");
        player.play(11);
        playCommandSent = true;
    }
    else if (strcmp(label, "street sign") == 0) {
        Serial.println("Playing track 12 (street sign)");
        player.play(12);
        playCommandSent = true;
    }
    else if (strcmp(label, "stop sign") == 0) {
        Serial.println("Playing track 13 (stop sign)");
        player.play(13);
        playCommandSent = true;
    }
    else if (strcmp(label, "parking meter") == 0) {
        Serial.println("Playing track 14 (parking meter)");
        player.play(14);
        playCommandSent = true;
    }
    else if (strcmp(label, "bench") == 0) {
        Serial.println("Playing track 15 (bench)");
        player.play(15);
        playCommandSent = true;
    }
    else if (strcmp(label, "bird") == 0) {
        Serial.println("Playing track 16 (bird)");
        player.play(16);
        playCommandSent = true;
    }
    else if (strcmp(label, "cat") == 0) {
        Serial.println("Playing track 17 (cat)");
        player.play(17);
        playCommandSent = true;
    }
    else if (strcmp(label, "dog") == 0) {
        Serial.println("Playing track 18 (dog)");
        player.play(18);
        playCommandSent = true;
    }
    else if (strcmp(label, "horse") == 0) {
        Serial.println("Playing track 19 (horse)");
        player.play(19);
        playCommandSent = true;
    }
    else if (strcmp(label, "sheep") == 0) {
        Serial.println("Playing track 20 (sheep)");
        player.play(20);
        playCommandSent = true;
    }
    else if (strcmp(label, "cow") == 0) {
        Serial.println("Playing track 21 (cow)");
        player.play(21);
        playCommandSent = true;
    }
    else if (strcmp(label, "elephant") == 0) {
        Serial.println("Playing track 22 (elephant)");
        player.play(22);
        playCommandSent = true;
    }
    else if (strcmp(label, "bear") == 0) {
        Serial.println("Playing track 23 (bear)");
        player.play(23);
        playCommandSent = true;
    }
    else if (strcmp(label, "zebra") == 0) {
        Serial.println("Playing track 24 (zebra)");
        player.play(24);
        playCommandSent = true;
    }
    else if (strcmp(label, "giraffe") == 0) {
        Serial.println("Playing track 25 (giraffe)");
        player.play(25);
        playCommandSent = true;
    }
    else if (strcmp(label, "hat") == 0) {
        Serial.println("Playing track 26 (hat)");
        player.play(26);
        playCommandSent = true;
    }
    else if (strcmp(label, "backpack") == 0) {
        Serial.println("Playing track 27 (backpack)");
        player.play(27);
        playCommandSent = true;
    }
    else if (strcmp(label, "umbrella") == 0) {
        Serial.println("Playing track 28 (umbrella)");
        player.play(28);
        playCommandSent = true;
    }
    else if (strcmp(label, "shoe") == 0) {
        Serial.println("Playing track 29 (shoe)");
        player.play(29);
        playCommandSent = true;
    }
    else if (strcmp(label, "eye glasses") == 0) {
        Serial.println("Playing track 30 (eye glasses)");
        player.play(30);
        playCommandSent = true;
    }
    else if (strcmp(label, "handbag") == 0) {
        Serial.println("Playing track 31 (handbag)");
        player.play(31);
        playCommandSent = true;
    }
    else if (strcmp(label, "tie") == 0) {
        Serial.println("Playing track 32 (tie)");
        player.play(32);
        playCommandSent = true;
    }
    else if (strcmp(label, "suitcase") == 0) {
        Serial.println("Playing track 33 (suitcase)");
        player.play(33);
        playCommandSent = true;
    }
    else if (strcmp(label, "frisbee") == 0) {
        Serial.println("Playing track 34 (frisbee)");
        player.play(34);
        playCommandSent = true;
    }
    else if (strcmp(label, "skis") == 0) {
        Serial.println("Playing track 35 (skis)");
        player.play(35);
        playCommandSent = true;
    }
    else if (strcmp(label, "snowboard") == 0) {
        Serial.println("Playing track 36 (snowboard)");
        player.play(36);
        playCommandSent = true;
    }
    else if (strcmp(label, "sports ball") == 0) {
        Serial.println("Playing track 37 (sports ball)");
        player.play(37);
        playCommandSent = true;
    }
    else if (strcmp(label, "kite") == 0) {
        Serial.println("Playing track 38 (kite)");
        player.play(38);
        playCommandSent = true;
    }
    else if (strcmp(label, "baseball bat") == 0) {
        Serial.println("Playing track 39 (baseball bat)");
        player.play(39);
        playCommandSent = true;
    }
    else if (strcmp(label, "baseball glove") == 0) {
        Serial.println("Playing track 40 (baseball glove)");
        player.play(40);
        playCommandSent = true;
    }
    else if (strcmp(label, "skateboard") == 0) {
        Serial.println("Playing track 41 (skateboard)");
        player.play(41);
        playCommandSent = true;
    }
    else if (strcmp(label, "surfboard") == 0) {
        Serial.println("Playing track 42 (surfboard)");
        player.play(42);
        playCommandSent = true;
    }
    else if (strcmp(label, "tennis racket") == 0) {
        Serial.println("Playing track 43 (tennis racket)");
        player.play(43);
        playCommandSent = true;
    }
    else if (strcmp(label, "bottle") == 0) {
        Serial.println("Playing track 44 (bottle)");
        player.play(44);
        playCommandSent = true;
    }
    else if (strcmp(label, "plate") == 0) {
        Serial.println("Playing track 45 (plate)");
        player.play(45);
        playCommandSent = true;
    }
    else if (strcmp(label, "wine glass") == 0) {
        Serial.println("Playing track 46 (wine glass)");
        player.play(46);
        playCommandSent = true;
    }
    else if (strcmp(label, "cup") == 0) {
        Serial.println("Playing track 47 (cup)");
        player.play(47);
        playCommandSent = true;
    }
    else if (strcmp(label, "fork") == 0) {
        Serial.println("Playing track 48 (fork)");
        player.play(48);
        playCommandSent = true;
    }
    else if (strcmp(label, "knife") == 0) {
        Serial.println("Playing track 49 (knife)");
        player.play(49);
        playCommandSent = true;
    }
    else if (strcmp(label, "spoon") == 0) {
        Serial.println("Playing track 50 (spoon)");
        player.play(50);
        playCommandSent = true;
    }
    else if (strcmp(label, "bowl") == 0) {
        Serial.println("Playing track 51 (bowl)");
        player.play(51);
        playCommandSent = true;
    }
    else if (strcmp(label, "banana") == 0) {
        Serial.println("Playing track 52 (banana)");
        player.play(52);
        playCommandSent = true;
    }
    else if (strcmp(label, "apple") == 0) {
        Serial.println("Playing track 53 (apple)");
        player.play(53);
        playCommandSent = true;
    }
    else if (strcmp(label, "sandwich") == 0) {
        Serial.println("Playing track 54 (sandwich)");
        player.play(54);
        playCommandSent = true;
    }
    else if (strcmp(label, "orange") == 0) {
        Serial.println("Playing track 55 (orange)");
        player.play(55);
        playCommandSent = true;
    }
    else if (strcmp(label, "broccoli") == 0) {
        Serial.println("Playing track 56 (broccoli)");
        player.play(56);
        playCommandSent = true;
    }
    else if (strcmp(label, "carrot") == 0) {
        Serial.println("Playing track 57 (carrot)");
        player.play(57);
        playCommandSent = true;
    }
    else if (strcmp(label, "hot dog") == 0) {
        Serial.println("Playing track 58 (hot dog)");
        player.play(58);
        playCommandSent = true;
    }
    else if (strcmp(label, "pizza") == 0) {
        Serial.println("Playing track 59 (pizza)");
        player.play(59);
        playCommandSent = true;
    }
    else if (strcmp(label, "donut") == 0) {
        Serial.println("Playing track 60 (donut)");
        player.play(60);
        playCommandSent = true;
    }
    else if (strcmp(label, "cake") == 0) {
        Serial.println("Playing track 61 (cake)");
        player.play(61);
        playCommandSent = true;
    }
    else if (strcmp(label, "chair") == 0) {
        Serial.println("Playing track 62 (chair)");
        player.play(62);
        playCommandSent = true;
    }
    else if (strcmp(label, "couch") == 0) {
        Serial.println("Playing track 63 (couch)");
        player.play(63);
        playCommandSent = true;
    }
    else if (strcmp(label, "potted plant") == 0) {
        Serial.println("Playing track 64 (potted plant)");
        player.play(64);
        playCommandSent = true;
    }
    else if (strcmp(label, "bed") == 0) {
        Serial.println("Playing track 65 (bed)");
        player.play(65);
        playCommandSent = true;
    }
    else if (strcmp(label, "mirror") == 0) {
        Serial.println("Playing track 66 (mirror)");
        player.play(66);
        playCommandSent = true;
    }
    else if (strcmp(label, "dining table") == 0) {
        Serial.println("Playing track 67 (dining table)");
        player.play(67);
        playCommandSent = true;
    }
    else if (strcmp(label, "window") == 0) {
        Serial.println("Playing track 68 (window)");
        player.play(68);
        playCommandSent = true;
    }
    else if (strcmp(label, "desk") == 0) {
        Serial.println("Playing track 69 (desk)");
        player.play(69);
        playCommandSent = true;
    }
    else if (strcmp(label, "toilet") == 0) {
        Serial.println("Playing track 70 (toilet)");
        player.play(70);
        playCommandSent = true;
    }
    else if (strcmp(label, "door") == 0) {
        Serial.println("Playing track 71 (door)");
        player.play(71);
        playCommandSent = true;
    }
    else if (strcmp(label, "tv") == 0) {
        Serial.println("Playing track 72 (tv)");
        player.play(72);
        playCommandSent = true;
    }
    else if (strcmp(label, "laptop") == 0) {
        Serial.println("Playing track 73 (laptop)");
        player.play(73);
        playCommandSent = true;
    }
    else if (strcmp(label, "mouse") == 0) {
        Serial.println("Playing track 74 (mouse)");
        player.play(74);
        playCommandSent = true;
    }
    else if (strcmp(label, "remote") == 0) {
        Serial.println("Playing track 75 (remote)");
        player.play(75);
        playCommandSent = true;
    }
    else if (strcmp(label, "keyboard") == 0) {
        Serial.println("Playing track 76 (keyboard)");
        player.play(76);
        playCommandSent = true;
    }
    else if (strcmp(label, "cell phone") == 0) {
        Serial.println("Playing track 77 (cell phone)");
        player.play(77);
        playCommandSent = true;
    }
    else if (strcmp(label, "microwave") == 0) {
        Serial.println("Playing track 78 (microwave)");
        player.play(78);
        playCommandSent = true;
    }
    else if (strcmp(label, "oven") == 0) {
        Serial.println("Playing track 79 (oven)");
        player.play(79);
        playCommandSent = true;
    }
    else if (strcmp(label, "toaster") == 0) {
        Serial.println("Playing track 80 (toaster)");
        player.play(80);
        playCommandSent = true;
    }
    else if (strcmp(label, "sink") == 0) {
        Serial.println("Playing track 81 (sink)");
        player.play(81);
        playCommandSent = true;
    }
    else if (strcmp(label, "refrigerator") == 0) {
        Serial.println("Playing track 82 (refrigerator)");
        player.play(82);
        playCommandSent = true;
    }
    else if (strcmp(label, "blender") == 0) {
        Serial.println("Playing track 83 (blender)");
        player.play(83);
        playCommandSent = true;
    }
    else if (strcmp(label, "book") == 0) {
        Serial.println("Playing track 84 (book)");
        player.play(84);
        playCommandSent = true;
    }
    else if (strcmp(label, "clock") == 0) {
        Serial.println("Playing track 85 (clock)");
        player.play(85);
        playCommandSent = true;
    }
    else if (strcmp(label, "vase") == 0) {
        Serial.println("Playing track 86 (vase)");
        player.play(86);
        playCommandSent = true;
    }
    else if (strcmp(label, "scissors") == 0) {
        Serial.println("Playing track 87 (scissors)");
        player.play(87);
        playCommandSent = true;
    }
    else if (strcmp(label, "teddy bear") == 0) {
        Serial.println("Playing track 88 (teddy bear)");
        player.play(88);
        playCommandSent = true;
    }
    else if (strcmp(label, "hair drier") == 0) {
        Serial.println("Playing track 89 (hair drier)");
        player.play(89);
        playCommandSent = true;
    }
    else if (strcmp(label, "toothbrush") == 0) {
        Serial.println("Playing track 90 (toothbrush)");
        player.play(90);
        playCommandSent = true;
    }
    else if (strcmp(label, "hair brush") == 0) {
        Serial.println("Playing track 91 (hair brush)");
        player.play(91);
        playCommandSent = true;
    }
    else {
        Serial.println("Label not recognized or no action defined.");
    }

    // --- Update state ONLY if play command was successfully sent ---
    if (playCommandSent) {
        isAudioPlaying = true; // Set the flag indicating playback has started
        lastAudioPlayTime = currentTime; // Update time debounce timer
        strncpy(lastPlayedLabel, label, MAX_LABEL_LENGTH); // Update last played label
        lastPlayedLabel[MAX_LABEL_LENGTH - 1] = '\0'; // Ensure null termination
        Serial.print("Updated last played label to: ");
        Serial.println(lastPlayedLabel);
    }
}

// --- ESP-NOW Callback ---
void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(message)) {
        memcpy(&message, incomingData, sizeof(message));
        
        if (message.isLabel) {
            // Handle label-based message
            message.label[MAX_LABEL_LENGTH - 1] = '\0'; // Ensure null termination
            Serial.print("Received label via ESP-NOW: '"); // Add quotes
            Serial.print(message.label); 
            Serial.print("' (Length: ");
            Serial.print(strlen(message.label)); // Use strlen for C-string
            Serial.println(")");
            playAudioForLabel(message.label);
        } else {
            // Handle command-based message (original functionality)
            Serial.print("Received command: ");
            Serial.println(message.command);

            uint32_t dimmedColor = strip1.Color(BRIGHTNESS, 0, 0);

            if (message.command == 1) {
                blinkLED(strip1, dimmedColor);
            }
            if (message.command == 2) {
                blinkLED(strip2, dimmedColor);
            }
        }
    } else {
        Serial.print("Received data of incorrect length: ");
        Serial.print(len);
        Serial.print(", expected: ");
        Serial.println(sizeof(message));
    }
}


void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Wireless Receiver Starting...");

    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address (this device): ");
    Serial.println(WiFi.macAddress());

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

    // Optional: Initialize lastPlayedLabel explicitly (already done globally, but safe)
    lastPlayedLabel[0] = '\0'; 

    isAudioPlaying = false; // Ensure flag is initially false
}


void loop() {
    // --- Check for DFPlayer Messages (like playback finished) ---
    if (player.available()) {
        uint8_t type = player.readType();
        //uint16_t value = player.read(); // May need this depending on library version/type

        switch (type) {
            case DFPlayerPlayFinished: // Track finished playing
                Serial.println(F("DFPlayer Finished Playing."));
                isAudioPlaying = false; // Clear the flag
                // We could potentially clear lastPlayedLabel here too if needed:
                // lastPlayedLabel[0] = '\0'; 
                break;
            case DFPlayerError: // Handle errors if needed
                 Serial.print(F("DFPlayer error: "));
                 // Serial.println(player.read()); // Print error code
                 isAudioPlaying = false; // Assume error stops playback
                 break;
             // Add other cases like DFPlayerOnline, DFPlayerCardOnline etc. if useful
            default:
                 // Ignore other message types for now
                 break;
        }
    }
    
    // Keep a small delay for stability, doesn't need to be large
    delay(20); 
}
