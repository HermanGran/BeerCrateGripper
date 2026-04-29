//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_GRIPPER_HPP
#define BEERCRATEGRIPPER_GRIPPER_HPP

#include <Arduino.h>
#include <Actuators/StepperMotor.hpp>
#include <Sensors/LimitSwitch.hpp>
#include <Sensors/CurrentSensor.hpp>

/**
 * @class Gripper
 *
 * @brief A class to control a robotic gripper controlled by a stepper motor,
 *        limit switch, and current sensor. Provides functionality for initialization,
 *        homing, latching, releasing, and state management.
 */
class Gripper {
public:

    Gripper();

    /**
     * @brief Initializes the object
     *
     * This function prepares the object for use, initializing the necessary parameters and object
     * used by this class, such as stepper motor, limit-switch, and current sensor.
     */
    void init();

    /**
     * @brief Initiates the homing sequence to calibrate the gripper's starting position.
     *
     * This method moves the gripper to its home position by driving the stepper motor
     * until the limit switch is triggered, then adjusts for precise positioning. The
     * stepper motor's outputs are enabled at the start and disabled upon completion.
     *
     * The homing process involves:
     * - Moving the stepper motor in reverse at a specific speed until the limit switch is pressed.
     * - Logging critical stages of the homing procedure for troubleshooting and verification.
     * - Once the limit switch has been pressed, backing off until the switch is released.
     * - Setting the home position using the StepperMotor's internal calibration method.
     *
     * @warning Manual intervention during the homing process may cause incorrect calibration
     */
    void homing();

    /**
     * @brief Moves the gripper to the latching position
     *
     * This method latches the gripper by running the stepper motor to a pre-defined position.
     *
     * @note Ensure the gripper is initialized and homed before calling this method
     *       to avoid inaccuracies in positioning.
     *
     * @warning Improper initialization or usage may result in incorrect positioning or
     *          potential damage to the gripper mechanism. If the stepper motor slips,
     *          run home to avoid damaging the gripper when releasing.
     */
    bool latch();

    /**
     * @brief Releases the gripper by moving it to the open position (home pos)
     *
     * This method moves the gripper to the home position and releasing the gripper.
     *
     * @note This method assumes the gripper has been initialized and is properly
     * calibrated to accurately respond to positional commands.
     */
    void release();

    /**
     * @brief Getter function for the stepper motor object within this class.
     *
     * @return A reference of the StepperMotor object.
     */
    StepperMotor& getStepper();

    /**
     * @brief Getter function for the current sensor object within this class.
     *
     * @return A reference of the CurrentSensor object.
     */
    CurrentSensor& getCurrentSensor();

    /**
     * @brief Getter function for the limit switch object within this class
     *
     * @return A reference of the LimitSwitch object
     */
    LimitSwitch& getLimitSwitch();

    // Gripper states
    enum class GripperState {
        IDLE, // Not homed, not running
        HOME, // Homed
        MOVING, // Normal movement, not in latch state
        OBSTACLE_DETECTED, // Obstacle detected before it should be hitting something
        TIGHTENING, // In the latch zone, hit something and slow down
        LATCHED, // Successfully latched
        FAILED // Reached and didn't detect anything
    };

    // Gripper actions
    enum class GripperAction : uint8_t {
        HOME,
        LATCH,
        RELEASE,
        IDLE // Move to the idle position
    };

    // Gripper state, default is idle
    GripperState gripperState_ = GripperState::IDLE;
    GripperAction gripperAction_ = GripperAction::IDLE;

    TaskHandle_t callerTaskHandle_ = nullptr;
    volatile bool tasksRunning_ = false;
private:

    // Objects used by the gripper
    StepperMotor stepper_;
    LimitSwitch limit_;
    CurrentSensor current_;

    TaskHandle_t stepperTaskHandle = nullptr;
    TaskHandle_t sensorTaskHandle  = nullptr;

    void moveToPosition(int position);
    static void sensorTaskWrapper(void* param);
    static void stepperTaskWrapper(void* param);

    // Various positions and steps
    static constexpr int stepsPerRev_ = 3200;
    static constexpr int latchZoneStart_ = stepsPerRev_ * 5;
    static constexpr int tightenSteps_ = stepsPerRev_ * 0.3;
    static constexpr int fullyExtended_ = stepsPerRev_ * 8;
    static constexpr int idlePos_ = stepsPerRev_ * 5;

    // Current threshold for detecting obstacles
    static constexpr float currentThreshold_ = 0.400;

    // Speed used for tightening when hitting the crate
    static constexpr int tightenSpeed_ = 400;
};

#endif //BEERCRATEGRIPPER_GRIPPER_HPP