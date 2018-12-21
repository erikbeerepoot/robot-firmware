//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//
#include <common/Common.h>
#include "../../Inc/motion/Motor.h"

Motor::Motor(GPIO_TypeDef *directionPinAPort,
             uint16_t directionPinA,
             GPIO_TypeDef *directionPinBPort,
             uint16_t directionPinB,
             TIM_HandleTypeDef *timer,
             uint16_t timerChannel,
             uint32_t compareValue) {
    a1Port = directionPinAPort;
    a2Port = directionPinBPort;
    a1Channel = directionPinA;
    a2Channel = directionPinB;
    pwmTimer = timer;
    pwmChannel = timerChannel;
    baseCompareValue = compareValue;
}

uint32_t Motor::compute_duty(int duty) {
    //clamp to valid values
    if (duty > 100) {
        duty = 100;
    } else if (duty < 0) {
        duty = 0;
    }
    return (uint32_t)(baseCompareValue * ((float)duty/100));
}


int Motor::setDuty(int newDuty) {
    if (newDuty > 100) {
        duty = 100;
    } else if (newDuty < 0) {
        duty = 0;
    } else {
        duty = newDuty;
    }

    uint32_t period = compute_duty(duty);
    __HAL_TIM_SET_COMPARE(pwmTimer, pwmChannel, period);

    return duty;
}

int Motor::getDuty(){
    return duty;
}

void Motor::_a1(GPIO_PinState state) {
    //PA0
    HAL_GPIO_WritePin(a1Port, a1Channel, state);
}

void Motor::_a2(GPIO_PinState state) {
    //PA1
    HAL_GPIO_WritePin(a2Port, a2Channel, state);
}

void Motor::run(bool reverse) {
    //PA0, PA1
    if(reverse) {
        _a1(LOW);
        _a2(HIGH);
    } else {
        _a1(HIGH);
        _a2(LOW);
    }
}

void Motor::stop(){
    _a1(LOW);
    _a2(LOW);
}