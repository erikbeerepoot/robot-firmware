//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_ROBOTSTATE_H
#define ROBOT_ROBOTSTATE_H

typedef struct MotionState {
    double v_x;
    double v_y;
} MotionState ;

typedef struct RobotState {
    MotionState motionState;
    float usonic = 0;
    float ir = 0;
    bool qs18 = false;
} RobotState;

#endif //ROBOT_ROBOTSTATE_H
