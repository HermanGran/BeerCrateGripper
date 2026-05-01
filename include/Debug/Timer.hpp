//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#ifndef BEERCRATEGRIPPER_TIMER_HPP
#define BEERCRATEGRIPPER_TIMER_HPP

#include <Arduino.h>

/**
 * @class Timer
 * @brief A class for managing and handling periodic timer interrupts.
 *
 * The Timer class utilizes hardware timing features to provide precise periodic
 * interrupt-based event handling. It supports initialization of a timer, checking
 * for elapsed time periods, and resetting the timer's state.
 */
class Timer {
public:
    explicit Timer(uint32_t periodUs);

    void init();
    bool isElapsed();  // returns true once per period, then resets
    void reset();

private:
    static void IRAM_ATTR onTimer();

    hw_timer_t*     timer_   = nullptr;
    const uint32_t  period_;
    volatile bool   elapsed_ = false;

    static Timer*   instance_;
};

#endif //BEERCRATEGRIPPER_TIMER_HPP
