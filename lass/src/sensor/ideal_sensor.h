//
// Created by violin on 2022/4/21.
//

#ifndef IDEAL_SENSOR_H
#define IDEAL_SENSOR_H

#include "sensor.h"

namespace Sensor {
    class IdealSensor : public Sensor {
    public:
        IdealSensor(int objectId, float x, float y, float z, float h, float rangeNear, float rangeFar, float fovH,
                    int maxObj);

        boost::python::tuple report() override;

    protected:
        int sensorId;
        float x;
        float y;
        float z;
        float h;
        float rangeNear;
        float rangeFar;
        float fovH;
        int maxObj;
    };
}

#endif
