//
// Created by Herman Hårstad Gran on 23/04/2026.
//

#include "Debug/Timer.hpp"

Timer* Timer::instance_ = nullptr;

Timer::Timer(uint32_t periodUs) : period_(periodUs) {}

void Timer::init() {
    instance_ = this;
    timer_ = timerBegin(0, 80, true);           // timer 0, div 80 → 1 µs ticks, count up
    timerAttachInterrupt(timer_, &Timer::onTimer, true);
    timerAlarmWrite(timer_, period_, true);     // autoreload
    timerAlarmEnable(timer_);
}

bool Timer::isElapsed() {
    if (elapsed_) {
        elapsed_ = false;
        return true;
    }
    return false;
}

void Timer::reset() {
    elapsed_ = false;
}

void IRAM_ATTR Timer::onTimer() {
    if (instance_) instance_->elapsed_ = true;
}
