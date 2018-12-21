//
// Created by Erik Beerepoot 😊 on 2018-12-20.
//

#include <common/RobotState.h>
#include <motion/PathPlanner.h>
#include <cmath>

PathPlanner::PathPlanner(Telemetry *telemetry,
                         Motor *l,
                         Motor *r) {
    this->telemetry = telemetry;
    this->l = l;
    this->r = r;

    l->stop();
    r->stop();
    l->setDuty(75);
    r->setDuty(65);
}

void PathPlanner::init(){}
void PathPlanner::terminate(){}

double k = 25;

RobotState PathPlanner::service(RobotState state) {
    static RobotState lastState;

    bool receivedCommand = telemetry->receiveCommand(state.motionState, &(state.motionState));
    if(!receivedCommand){
        //plan a path
        if (!state.qs18) {
            state.motionState = STOPPED;
        } else {
            if (state.ir < 20) {
                //execute turn
                l->run(false);
                r->run(true);
                HAL_Delay(4000);
            } else {
                if (state.usonic > 120 && l->getDuty() > 5) {
                    float distanceFactor = (state.usonic - 120)  / 5;
                    l->setDuty(l->getDuty() - 4*distanceFactor);
                    r->setDuty(r->getDuty() + 4*distanceFactor);
                } else if (state.usonic < 100 && l->getDuty() < 55) {
                    float distanceFactor = (state.usonic)  / 5;
                    l->setDuty(l->getDuty() + 4*distanceFactor);
                    r->setDuty(r->getDuty() - 4*distanceFactor);
                } else {
                    l->setDuty(25);
                    r->setDuty(20);
                }
            }
        }
    } else {
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }
    
    switch (state.motionState) {
        case STRAIGHT:
            l->run(false);
            r->run(false);
            break;
        case STOPPED:
            l->stop();
            r->stop();
            break;
        case LEFT:
            l->run(true);
            r->run(false);
            break;
        case RIGHT:
            l->run(false);
            r->run(true);
            break;
        case REVERSE:
            l->run(true);
            r->run(true);
            break;
        case CHOOSE_DIRECTION:
            break;
        case FAULT:
            l->stop();
            r->stop();
            break;
    }
    lastState = state;
    return state;
}

