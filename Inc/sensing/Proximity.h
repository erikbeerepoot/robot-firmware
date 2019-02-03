//
// Created by Erik Beerepoot 😊 on 2018-12-19.
//

#ifndef ROBOT_PROXIMITY_H
#define ROBOT_PROXIMITY_H

#include <stm32f401xc.h>

class Proxmity {
public:
    virtual float readDistance() = 0;
};

#endif //ROBOT_PROXIMITY_H
