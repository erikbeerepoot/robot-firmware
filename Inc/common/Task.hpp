//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "../common/RobotState.h"

class Task {
    /**
     * Loop called to "service" task
     */
    virtual void service(RobotState state) = 0;

    /**
     * Called to allow task to perform initialization
     */
    virtual void init() = 0;

    /**
     * Called to allow task to shutdown/release any resources.
     * After this runs, service() will no longer be called.
     */
    virtual void terminate() = 0;
};


#endif //ROBOT_TASK_H
