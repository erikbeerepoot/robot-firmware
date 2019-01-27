//
// Created by Erik Beerepoot 😊 on 2018-12-20.
//

#include "Proximity.h"

#ifndef ROBOT_MAXBOTIXRANGEFINDER_H
#define ROBOT_MAXBOTIXRANGEFINDER_H

#endif //ROBOT_MAXBOTIXRANGEFINDER_H

class MaxbotixRangefinder: Proxmity {
public:
    MaxbotixRangefinder();
    float readDistance() override;
private:
    float computeDistance(uint32_t delta);
};