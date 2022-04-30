//
// Created by violin on 2022/4/30.
//

#ifndef ROAD_SENSOR_H
#define ROAD_SENSOR_H

#include "sensor.h"

namespace Sensor {
    class RoadSensor : public Sensor {
    public:
        RoadSensor(int objectId);
        ~RoadSensor() override;

        boost::python::tuple report() override;

    protected:
        int posHandle;
    };
}


#endif //ROAD_SENSOR_H
