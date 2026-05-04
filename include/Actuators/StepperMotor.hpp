//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_STEPPERMOTOR_HPP
#define BEERCRATEGRIPPER_STEPPERMOTOR_HPP

#include <Arduino.h>
#include <AccelStepper.h>
#include <Sensors/CurrentSensor.hpp>


/**
 * @class StepperMotor
 * @brief Represents a stepper motor controlled using the AccelStepper library, with support for hardware initialization, speed control, position control, and acceleration management.
 *
 * The StepperMotor class provides methods to initialize the stepper motor, control its motion, and interact with its positional or running state. It uses the AccelStepper library for precise control of step and direction signals and includes a current sensor for collision detection.
 */
class StepperMotor {
public:

    /**
     * @brief Constructor for the stepper motor class.
     *
     * This class has to take in the below parameters to function properly, the current sensor
     * is used to detect of the motor is colliding with something.
     *
     * @param EN_PIN Enable pin for the motor driver
     * @param DIR_PIN Direction pin for the motor driver
     * @param STEP_PIN Step pin for the motor driver
     */
    StepperMotor(int8_t enPin, int8_t dirPin, int8_t stepPin, int8_t ms1Pin, int8_t ms2Pin);

    /**
     * @brief Initializes the stepper motor and its settings.
     *
     * Configures the stepper motor hardware by initializing its control pins,
     * setting the default speed, acceleration, and pin configuration. It also disables the motor outputs
     * to ensure safety until the motor is explicitly enabled. This avoids unnecessary power usage and prevents
     * the motor driver board from overheating.
     *
     * @note This method must be invoked before any motion-related functions to properly initialize the
     * stepper motor hardware.
     */
    void init();

    void setSpeed(float speed);

    /**
     * @brief Function for manually setting the motor soeed
     *
     * This function is used to setting the speed manually, mainly used inside the homing function
     * in the gripper class.
     *
     * @param speed Speed for the motor to drive with
     */
    void setMaxSpeed(float speed);

    /**
     * @brief Function for setting acceleration of the stepper motor
     *
     * This function sets the acceleration of the stepper motor. This is mainly used when using the runToPosition
     * function within the AccelStepper library, which accelerates and decelerates when running.
     *
     * @param acceleration Acceleration of the stepper motor
     */
    void setAcceleration(float acceleration);

    /**
     * @brief Function for setting a home position
     *
     * This function is used for homing the stepper motor, places the current position of the motor
     * as it's new home (0).
     */
    void setHomePos();

    /**
     * @brief Executes the stepping motion of the stepper motor if it is currently running.
     *
     * This method checks if the stepper motor is in motion. If the motor is running,
     * it calls the `run()` function of the `AccelStepper` instance to advance the motor
     * position based on the current step instructions. It continuously monitors the
     * remaining distance to the target. Once the motor reaches its target position and no
     * further steps are required, it stops the motor by disabling its outputs and updates
     * the internal `running_` flag to `false`.
     *
     * @note This method should be called periodically in a loop or task to ensure
     * smooth movement and precise control of the stepper motor.
     */
    void run();

    /**
     * @brief Stops the motion of the stepper motor.
     *
     * This method halts the current movement of the stepper motor immediately by invoking the
     * `stop()` function of the underlying `AccelStepper` instance. It does not reset the target
     * position but ensures the motor stops moving at its current step. This can be useful in
     * emergency cases or when an immediate stop is required.
     *
     * @note Ensure that stopping the motor does not cause abrupt operational issues such as
     * mechanical stress or position misalignments in the motor's system.
     */
    void stop();

    /**
     * @brief Runs the stepper motor to a position
     *
     * This function takes in a position in number of steps to move. The position is internally stored. It utilizes a smooth stop and start.
     *
     * @note A full rotation is 3200 steps. To move the motor one full rotation, send 3200 as a position
     *
     * @param position number of steps the motor moves
     */
    void runToPosition(int position);

    /**
     * @brief A boolean value that is updated if the motor is running or not
     *
     * @return Boolean status, running or stopped
     */
    bool isRunning();

    void setMicroStepping(int8_t microStepping = 8) const;

    void enableOutputs();
    void disableOutputs();

    void setSpeedParams(float speed, float acceleration);

    void runSpeed();

    int getPosition();

    void setCurrentPosition(int position);

private:
    // Objects used by this class
    AccelStepper stepper_;

    // Pins for the motor driver
    const int8_t enPin_;
    const int8_t dirPin_;
    const int8_t stepPin_;
    const int8_t ms1Pin_;
    const int8_t ms2Pin_;

    const int fullSteps = 200;
    int stepsPerRevolution_ = 0;
};

#endif //BEERCRATEGRIPPER_STEPPERMOTOR_HPP