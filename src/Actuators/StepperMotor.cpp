//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#include "Actuators/StepperMotor.hpp"
#include <HardwareSerial.h>
#include <Debug/Logger.hpp>

StepperMotor::StepperMotor(const int8_t EN_PIN, const int8_t DIR_PIN, const int8_t STEP_PIN, const int8_t RX_PIN, const int8_t TX_PIN)
    :   stepper_(AccelStepper::DRIVER, STEP_PIN, DIR_PIN),
        enPin_(EN_PIN),
        dirPin_(DIR_PIN),
        stepPin_(STEP_PIN),
        rxPin_(RX_PIN),
        txPin_(TX_PIN)
{}

// ─────────────────────────────────────────────
// Raw UART diagnostic — no library involved
// ─────────────────────────────────────────────
uint8_t StepperMotor::calcCRC(uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

void StepperMotor::runUARTDiagnostic() {
    logger.logf("=== TMC2208 Raw UART Diagnostic ===");
    logger.logf("Wiring should be:");
    logger.logf("  TX (pin %d) --[1k ohm]--+-- PDN_UART", txPin_);
    logger.logf("  RX (pin %d) ------------+", rxPin_);
    logger.logf("  CLK pin tied to GND");
    logger.logf("  VCC_IO = 3.3V");
    logger.logf("  ENN pin = LOW");
    logger.logf("");

    // Flush anything stale
    while (Serial2.available()) Serial2.read();
    delay(10);

    // ── Test 1: Send 0xAA repeatedly and check RX sees it ──
    // This confirms TX→RX loopback is working at all
    logger.logf("Test 1: TX->RX loopback check");
    Serial2.write(0xAA);
    Serial2.flush();
    delay(10);
    if (Serial2.available()) {
        uint8_t echo = Serial2.read();
        logger.logf("  Loopback OK — sent 0xAA, got 0x%02X", echo);
        if (echo != 0xAA)
            logger.logf("  WARNING: echo mismatch, check wiring");
    } else {
        logger.logf("  FAIL — nothing received at all");
        logger.logf("  TX and RX are not connected together");
        logger.logf("  Check your wiring: both pins must share one wire to PDN_UART");
        return; // No point continuing
    }
    while (Serial2.available()) Serial2.read();
    delay(10);

    // ── Test 2: Send valid read request for IOIN (0x06) ──
    // IOIN contains a version byte (0x20) we can verify
    logger.logf("");
    logger.logf("Test 2: Read IOIN register (0x06)");
    logger.logf("  Expected: 12 bytes total (4 echo + 8 reply)");
    logger.logf("  Version byte (reply[3]) should be 0x20");

    uint8_t req[4];
    req[0] = 0x05;       // sync nibble
    req[1] = 0x00;       // node address
    req[2] = 0x06;       // IOIN register, read bit = 0
    req[3] = calcCRC(req, 3);

    logger.logf("  Sending: 0x%02X 0x%02X 0x%02X 0x%02X",
                req[0], req[1], req[2], req[3]);

    while (Serial2.available()) Serial2.read();

    Serial2.write(req, 4);
    Serial2.flush();

    // Wait generously for echo + reply
    delay(20);

    int bytesAvailable = Serial2.available();
    logger.logf("  Bytes received after 20ms: %d", bytesAvailable);

    uint8_t buf[16];
    uint8_t count = 0;
    while (Serial2.available() && count < 16) {
        buf[count++] = Serial2.read();
    }

    // Print everything raw
    String raw = "  Raw bytes: ";
    for (int i = 0; i < count; i++) {
        char tmp[8];
        sprintf(tmp, "0x%02X ", buf[i]);
        raw += tmp;
    }
    logger.logf(raw.c_str());

    // Interpret the result
    if (count == 0) {
        logger.logf("");
        logger.logf("  RESULT: 0 bytes — chip not responding at all");
        logger.logf("  Most likely causes:");
        logger.logf("    1. PDN_UART solder bridge not made on bottom of board");
        logger.logf("    2. VCC_IO not connected or wrong voltage");
        logger.logf("    3. Wrong TX/RX pin numbers");

    } else if (count == 4) {
        logger.logf("");
        logger.logf("  RESULT: Only echo received, no chip reply");
        logger.logf("  Your TX->RX wiring works but chip is silent");
        logger.logf("  Most likely causes:");
        logger.logf("    1. PDN_UART solder bridge not made — most common!");
        logger.logf("    2. ENN pin is HIGH (motor disabled)");
        logger.logf("    3. VS (motor supply) not powered");

    } else if (count == 12) {
        // We got echo (4) + reply (8)
        uint8_t* reply = buf + 4;

        logger.logf("");
        logger.logf("  Echo:  0x%02X 0x%02X 0x%02X 0x%02X",
                    buf[0], buf[1], buf[2], buf[3]);
        logger.logf("  Reply: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                    reply[0], reply[1], reply[2], reply[3],
                    reply[4], reply[5], reply[6], reply[7]);

        uint8_t expectedCRC = calcCRC(reply, 7);
        bool crcOK = (expectedCRC == reply[7]);

        logger.logf("  CRC: got 0x%02X, expected 0x%02X — %s",
                    reply[7], expectedCRC, crcOK ? "OK" : "MISMATCH");

        uint8_t version = reply[3]; // VERSION is bits 31-24 of IOIN
        logger.logf("  Version byte: 0x%02X (expect 0x20)", version);

        if (crcOK && version == 0x20) {
            logger.logf("");
            logger.logf("  RESULT: TMC2208 UART is working correctly!");
            logger.logf("  You can now use the TMC library.");
        } else if (!crcOK) {
            logger.logf("");
            logger.logf("  RESULT: Got 12 bytes but CRC is wrong");
            logger.logf("  Possible causes:");
            logger.logf("    1. Electrical noise — try shorter wires");
            logger.logf("    2. Logic level mismatch (VCC_IO != ESP32 IO voltage)");
            logger.logf("    3. Missing 1k resistor between TX and shared wire");
        }

    } else {
        logger.logf("");
        logger.logf("  RESULT: Unexpected byte count (%d)", count);
        logger.logf("  Possible causes:");
        logger.logf("    1. Noise on the line");
        logger.logf("    2. Missing 1k ohm resistor causing signal corruption");
        logger.logf("    3. Baud rate mismatch");
    }

    logger.logf("");
    logger.logf("=== Diagnostic complete ===");
}

void StepperMotor::init() {
    Serial2.begin(115200, SERIAL_8N1, rxPin_, txPin_);
    delay(100);

    // Run hardware diagnostic first, before any library calls
    runUARTDiagnostic();

    // Only proceed with library init if diagnostic passes
    // (comment this out if diagnostic shows failure until hardware is fixed)
    tmc_ = new TMC2208Stepper(&Serial2, rSense_);
    tmc_->begin();
    tmc_->pdn_disable(true);
    tmc_->mstep_reg_select(true);
    tmc_->rms_current(800);
    tmc_->microsteps(8);
    tmc_->ihold(8);
    tmc_->pwm_autoscale(true);
    delay(500);

    const uint8_t version = tmc_->version();
    if (version == 0x20) {
        logger.logf("TMC2208 library OK — version: 0x%02X", version);
    } else {
        logger.logf("TMC2208 library FAILED — got: 0x%02X, CRCerror: %d",
                    version, (int)tmc_->CRCerror);
    }

    pinMode(enPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    stepper_.setMaxSpeed(4000);
    stepper_.setAcceleration(2000);
    stepper_.setEnablePin(enPin_);
    stepper_.setPinsInverted(false, false, true);
    stepper_.disableOutputs();
}

// ── rest of your methods unchanged ──

void StepperMotor::setSpeed(const int speed) {
    stepper_.setMaxSpeed(speed);
}

void StepperMotor::setAcceleration(const int acceleration) {
    stepper_.setAcceleration(acceleration);
}

void StepperMotor::setHomePos() {
    stepper_.setCurrentPosition(0);
}

AccelStepper* StepperMotor::getAccelStepper() {
    return &stepper_;
}

void StepperMotor::stop() {
    stepper_.stop();
}

void StepperMotor::runToPosition(const int position) {
    stepper_.enableOutputs();
    stepper_.moveTo(position);
    running_ = true;
}

void StepperMotor::run() {
    if (running_) {
        stepper_.run();
        if (stepper_.distanceToGo() == 0) {
            running_ = false;
            stepper_.disableOutputs();
        }
    }
}

bool StepperMotor::isRunning() const {
    return running_;
}