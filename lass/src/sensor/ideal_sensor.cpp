//
// Created by violin on 2022/4/21.
//

#include <boost/python.hpp>
#include "ideal_sensor.h"
#include "esmini/esminiLib.hpp"
#include "utils/utils.h"

namespace Sensor {
    IdealSensor::IdealSensor(int objectId, float x, float y, float z, float h, float rangeNear, float rangeFar,
                             float fovH, int maxObj) : Sensor(objectId, "IdealSensor"), x(x), y(y), z(z), h(h), rangeNear(rangeNear),
                                                       rangeFar(rangeFar), fovH(fovH), maxObj(maxObj) {
        sensorId = SE_AddObjectSensor(objectId, x, y, z, h, rangeNear, rangeFar, fovH, maxObj);
        SE_ViewSensorData(objectId);
    }

    boost::python::tuple IdealSensor::report() {
        boost::python::list id_list;
        boost::python::list data;
        int detectedObjectList[maxObj];
        int detectedObjectNums = SE_FetchSensorObjectList(sensorId, detectedObjectList);
        if (detectedObjectNums > 0)
        {
            SE_ScenarioObjectState egoState;
            SE_GetObjectState(objectId, &egoState);
            for (int i = 0; i < detectedObjectNums; i++)
            {
                id_list.append(detectedObjectList[i]);
                data.append(Utils::getCurrentObs(detectedObjectList[i]));
            }
        }
        return boost::python::make_tuple(sensorType, id_list, data);
    }
}
