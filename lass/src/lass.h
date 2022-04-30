#ifndef LASS_H
#define LASS_H

#include <map>
#include <boost/python.hpp>
#include "osi/osi_groundtruth.pb.h"
#include "osi/osi_sensordata.pb.h"
#include "controller/controller.h"
#include "sensor/sensor.h"

#define OSI_SERIALIZE_BUFFER_SIZE 20000

struct Vehicle {
    int id;
    std::string name;
    Controller::Controller *controller;
    std::vector<Sensor::Sensor *> sensor;
};

class Lass {
public:
    Lass(const char *xoscString, int display);

    ~Lass();

    boost::python::list observe();

    boost::python::dict perceive();

    boost::python::tuple step(boost::python::dict action);

    boost::python::dict vehicleDict();

private:
    float dt;
    std::string xoscString;
    std::map<int, struct Vehicle *> vehicles;
    osi3::GroundTruth* osi_groundTruth;
    osi3::SensorData* osi_sensorData;
};

#endif
