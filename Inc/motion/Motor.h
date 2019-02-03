//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <stm32f401xc.h>
#include <stm32f4xx_hal.h>

class Motor {
public:
    Motor(GPIO_TypeDef *directionPinAPort,
          uint16_t directionPinA,
          GPIO_TypeDef *directionPinBPort,
          uint16_t directionPinB,
          TIM_HandleTypeDef *pwmTimer,
          uint16_t pwmChannel,
          uint32_t compareValue);

    int setDuty(int duty);
    int getDuty();

    void run(bool reverse);
    void stop();
private:
    uint32_t compute_duty(int duty);
    void _a1(GPIO_PinState state);
    void _a2(GPIO_PinState state);

    int duty = 0;
    uint32_t baseCompareValue = 0;

    // Define direction pins
    GPIO_TypeDef *a1Port;
    GPIO_TypeDef *a2Port;
    uint16_t a1Channel;
    uint16_t a2Channel;
    TIM_HandleTypeDef *pwmTimer;
    uint16_t pwmChannel;
};

#endif //ROBOT_MOTOR_H
