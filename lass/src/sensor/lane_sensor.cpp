//
// Created by violin on 2022/4/23.
//

#include "lane_sensor.h"
#include <boost/python.hpp>
#include "esmini/esminiLib.hpp"

namespace Sensor {
    LaneSensor::LaneSensor(int objectId, float lookaheadDistance, int lookaheadMode, bool inRoadDrivingDirection)
            : Sensor(objectId, "LaneSensor"),
            distance(lookaheadDistance), mode(lookaheadMode), inRoadDrivingDirection(inRoadDrivingDirection) {}

    boost::python::tuple LaneSensor::report() {
        SE_RoadInfo roadInfo;
        SE_GetRoadInfoAtDistance(objectId, distance, &roadInfo, mode, inRoadDrivingDirection);
        return boost::python::make_tuple(sensorType, roadInfo);
    }
}
