//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_ROBOTSTATE_H
#define ROBOT_ROBOTSTATE_H

typedef enum MotionState {
    STRAIGHT = 0,
    LEFT,
    RIGHT,
    STOPPED,
    REVERSE,
    CHOOSE_DIRECTION,
    FAULT
} MotionState ;

typedef struct RobotState {
    MotionState motionState = STOPPED;
    float usonic = 0;
    float ir = 0;
    bool qs18 = false;
} RobotState;

#endif //ROBOT_ROBOTSTATE_H
