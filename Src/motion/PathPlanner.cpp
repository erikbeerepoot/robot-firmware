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

//    l->stop();
//    r->stop();
//    l->setDuty(35);
//    r->setDuty(25);
}

void PathPlanner::init() {}


double k = 25;

RobotState PathPlanner::service(RobotState state) {
//    static RobotState lastState;
//
//    bool receivedCommand = telemetry->receiveCommand());
//    if (receivedCommand) {
//        double v_x = state.motionState.v_x;
//        double v_y = state.motionState.v_y;
//
//        int forward = int(v_y * 50);
//        int left = 0;
//        int right = 0;
//        if (v_x >= 0) {
//            right = int(fabs(v_y)*50);
//        } else if (v_x < 0) {
//            left = int(fabs(v_x)*50);
//        }
//        l->setDuty(forward + left);
//        r->setDuty(forward + right);
//        l->run(false);
//        r->run(false);
//    } else {
//        lastState = state;
//    }
    return state;
}

