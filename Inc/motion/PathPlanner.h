//
// Created by Erik Beerepoot 😊 on 2018-12-20.
//

#include <common/Task.hpp>
#include <comm/Telemetry.hpp>
#include "Motor.h"

#ifndef ROBOT_PATHPLANNER_H
#define ROBOT_PATHPLANNER_H

#endif //ROBOT_PATHPLANNER_H

class PathPlanner: Task {
public:
    PathPlanner(Telemetry *telemetry,
                Motor *l,
                Motor *r);
    void init();
    void terminate();
    RobotState service(RobotState state) override;
private:
    Telemetry *telemetry;
    Motor *l;
    Motor *r;
};