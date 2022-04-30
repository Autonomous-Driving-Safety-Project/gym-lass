//
// Created by violin on 2022/4/23.
//

#ifndef LANE_SENSOR_H
#define LANE_SENSOR_H

#include "sensor.h"

namespace Sensor {
    class LaneSensor : public Sensor {
    public:
        LaneSensor(int objectId, float lookaheadDistance, int lookaheadMode, bool inRoadDrivingDirection);

        boost::python::tuple report() override;

    protected:
        float distance;
        int mode;
        bool inRoadDrivingDirection;
    };
}


#endif //LANE_SENSOR_H
