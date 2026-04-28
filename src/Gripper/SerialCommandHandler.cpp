//
// Created by Herman Hårstad Gran on 24/03/2026.
//

#include "Gripper/SerialCommandHandler.hpp"

SerialCommandHandler::SerialCommandHandler(Gripper& gripper)
    : gripper_(gripper)
{
}

void SerialCommandHandler::init(unsigned long baud) {
    Serial.begin(baud);
    while (!Serial) {
        ; // wait for serial (ESP32 usually doesn't need this, but safe)
    }

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_TASK_WDT) {
        Serial.println("RESET REASON: Task Watchdog!");
    } else if (reason == ESP_RST_INT_WDT) {
        Serial.println("RESET REASON: Interrupt Watchdog!");
    } else {
        Serial.println("RESET REASON: " + String(reason));
    }

    Serial.println("Ready. Commands: open, close, home");
}

void SerialCommandHandler::update() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (buffer_.length() > 0) {
                processCommand(buffer_);
                buffer_ = "";
            }
        } else {
            buffer_ += c;
        }
    }
}

void SerialCommandHandler::processCommand(const String& cmd) {
    if (cmd == "latch") {
        Serial.println("Gripepr latching...");
        gripper_.latch();
    }
    else if (cmd == "release") {
        Serial.println("Gripper releasing...");
        gripper_.release();
    }
    else if (cmd == "home") {
        Serial.println("Homing gripper...");
        gripper_.homing();
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
    }
}