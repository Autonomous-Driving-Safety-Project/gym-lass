#ifndef SENSOR_H
#define SENSOR_H

#include <boost/python.hpp>
#include <utility>

namespace Sensor {
    class Sensor {
    public:
        explicit Sensor(int objectId, std::string type) : objectId(objectId), sensorType(std::move(type)) {};
        virtual ~Sensor() = default;
        virtual boost::python::tuple report() = 0;
        const std::string sensorType;

    protected:
        int objectId;
    };
}

#endif
