//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_ROBOTSTATE_H
#define ROBOT_ROBOTSTATE_H

typedef enum RobotState {
    STRAIGHT = 0,
    LEFT,
    RIGHT,
    STOPPED,
    REVERSE,
    FAULT
} RobotState;

#endif //ROBOT_ROBOTSTATE_H
