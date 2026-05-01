//
// Created by Herman Hårstad Gran on 23/03/2026.
//

#ifndef BEERCRATEGRIPPER_LIMITSWITCH_HPP
#define BEERCRATEGRIPPER_LIMITSWITCH_HPP

class LimitSwitch {
public:
    explicit LimitSwitch(int pin);

    void init() const noexcept;

    bool isPressed() const noexcept;

private:
    int pin_;
};

#endif //BEERCRATEGRIPPER_LIMITSWITCH_HPP