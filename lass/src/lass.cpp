#include <cstdlib>
#include <boost/python.hpp>
#include <vector>
#include <array>
#include <cstring>
#include "rapidxml/rapidxml.hpp"
#include "esmini/esminiLib.hpp"
#include "esmini/esminiRMLib.hpp"
#include "osi/osi_common.pb.h"
#include "osi/osi_object.pb.h"
#include "osi/osi_groundtruth.pb.h"
#include "osi/osi_sensordata.pb.h"
#include "osi/osi_version.pb.h"
#include "lass.h"
#include "controller/naive.h"
#include "sensor/ideal_sensor.h"
#include "sensor/lane_sensor.h"
#include "sensor/road_sensor.h"
#include "utils/utils.h"
#include <iostream>

using namespace boost::python;

struct ScenarioException : public std::exception {
    const char *what() const throw() {
        return "scenario parse exception";
    }
};

Lass::Lass(const char *xoscString, int display) :
        xoscString(xoscString), dt(0.01) {
    Py_Initialize();
    SE_LogToConsole(false);
    SE_SetLogFilePath("");
    SE_CollisionDetection(true);
    display = !!display;
    SE_InitWithString(this->xoscString.c_str(), 0, display, 0, 0);
    RM_Init(SE_GetODRFilename());

    // Initialize OSI
    SE_UpdateOSIGroundTruth();

    // get each vehicle name and id
    int vehicle_num = SE_GetNumberOfObjects();
    for (int i = 0; i < vehicle_num; i++) {
        auto *v = new struct Vehicle;
        v->id = SE_GetId(i);
        v->name = SE_GetObjectName(v->id);
        v->controller = nullptr;
        vehicles.insert(std::pair<int, Vehicle *>(v->id, v));
    }

    // handle controllers and sensors
    char *cstr = new char[this->xoscString.size() + 1];
    strcpy(cstr, this->xoscString.c_str());
    rapidxml::xml_document<> doc;
    doc.parse<0>(cstr);
    rapidxml::xml_node<> *entities = doc.first_node()->first_node("Entities");
    if (entities) {
        rapidxml::xml_node<> *objNode = entities->first_node("ScenarioObject");
        while (objNode) {
            rapidxml::xml_attribute<> *vehicleName;
            vehicleName = objNode->first_attribute("name");
            if (vehicleName) {
                std::string vehicleNameStr(vehicleName->value());
                struct Vehicle *pVehicle = nullptr;
                for (auto i: vehicles) {
                    if (i.second->name == vehicleNameStr) {
                        pVehicle = i.second;
                        break;
                    }
                }

                // get controller
                try {
                    rapidxml::xml_attribute<> *controllerName = nullptr;
                    rapidxml::xml_node<> *pNode = objNode->first_node("ObjectController");
                    if (!pNode) throw ScenarioException();
                    pNode = pNode->first_node("Controller");
                    if (!pNode) throw ScenarioException();
                    pNode = pNode->first_node("Properties");
                    if (!pNode) throw ScenarioException();
                    rapidxml::xml_node<> *prop = pNode->first_node("Property");
                    while (prop) {
                        rapidxml::xml_attribute<> *attr = prop->first_attribute("name");
                        if (attr && !strcmp(attr->value(), "baseController")) {
                            controllerName = prop->first_attribute("value");
                            break;
                        }
                        prop = prop->next_sibling("Property");
                    }
                    if (controllerName) {
                        if (!strcmp(controllerName->value(), "Naive")) {
                            pVehicle->controller = new Controller::Naive(pVehicle->id, dt);
                            // TODO: support parameters
                        }
                        // If more controllers are implemented, add here
                    }
                }
                catch (ScenarioException &e) {
                    // do nothing
                }

                // get sensors
                try {
                    rapidxml::xml_node<> *sensorProp = nullptr;
                    rapidxml::xml_node<> *pNode = objNode->first_node("ObjectSensor");
                    if (!pNode) throw ScenarioException();
                    rapidxml::xml_node<> *pSensor = pNode->first_node("Sensor");
                    while (pSensor) {
                        rapidxml::xml_node<> *pProps = pSensor->first_node("Properties");
                        if (!pProps) throw ScenarioException();
                        rapidxml::xml_node<> *prop = pProps->first_node("Property");
                        while (prop) {
                            rapidxml::xml_attribute<> *attr = prop->first_attribute("name");
                            if (attr && !strcmp(attr->value(), "sensorType")) {
                                sensorProp = prop;
                                break;
                            }
                            prop = prop->next_sibling("Property");
                        }
                        rapidxml::xml_attribute<> *sensorType = sensorProp->first_attribute("value");
                        if (pVehicle && sensorType && !strcmp(sensorType->value(), "IdealSensor")) {
                            Sensor::Sensor *s = new Sensor::IdealSensor(
                                    pVehicle->id,
                                    atof(sensorProp->first_attribute("x")->value()),
                                    atof(sensorProp->first_attribute("y")->value()),
                                    atof(sensorProp->first_attribute("z")->value()),
                                    atof(sensorProp->first_attribute("h")->value()),
                                    atof(sensorProp->first_attribute("rangeNear")->value()),
                                    atof(sensorProp->first_attribute("rangeFar")->value()),
                                    atof(sensorProp->first_attribute("fovH")->value()),
                                    atof(sensorProp->first_attribute("maxObj")->value())
                            );
                            pVehicle->sensor.emplace_back(s);
                        } else if (pVehicle && sensorType && !strcmp(sensorType->value(), "LaneSensor")) {
                            bool inRoadDirection = false;
                            rapidxml::xml_attribute<> *dir_attr = sensorProp->first_attribute("inRoadDirection");
                            if (dir_attr != nullptr) {
                                std::string str = dir_attr->value();
                                inRoadDirection = true;
                                if (strcasecmp(str.c_str(), "false") == 0 || strcmp(str.c_str(), "0") == 0) {
                                    inRoadDirection = false;
                                }
                            }
                            Sensor::Sensor *s = new Sensor::LaneSensor(
                                    pVehicle->id,
                                    atof(sensorProp->first_attribute("lookaheadDistance")->value()),
                                    atoi(sensorProp->first_attribute("lookaheadMode")->value()),
                                    inRoadDirection
                            );
                            pVehicle->sensor.emplace_back(s);
                        } else if (pVehicle && sensorType && !strcmp(sensorType->value(), "RoadSensor")) {
                            Sensor::Sensor *s = new Sensor::RoadSensor(pVehicle->id);
                            pVehicle->sensor.emplace_back(s);
                        }
                        // If more sensors are implemented, add here

                        pSensor = pSensor->next_sibling("Sensor");
                    }
                }
                catch (ScenarioException &e) {
                    // do nothing
                }
            }
            objNode = objNode->next_sibling("ScenarioObject");
        }
    }
    delete[] cstr;
}

Lass::~Lass() {
    for (const auto &i: vehicles) {
        delete i.second->controller;
        for (auto ii: i.second->sensor) {
            delete ii;
        }
        delete i.second;
    }
    SE_Close();
    RM_Close();
}

list Lass::observe() {
    return Utils::getCurrentObs();
}

dict Lass::perceive() {
    dict perception;
    for (const auto &i: vehicles) {
        list sensorData;
        for (auto sensor: i.second->sensor) {
            sensorData.append(sensor->report());
        }
        perception[i.first] = sensorData;
    }
    return perception;
}

tuple Lass::step(dict actions) {
    // handle vehicle changes
    // Stage 1: delete not exist vehicles
    for (const auto &i: vehicles) {
        SE_ScenarioObjectState objState; // useless
        if (SE_GetObjectState(i.first, &objState) == -1) {
            delete i.second;
            vehicles.erase(i.first); // This object does not exist anymore
        }
    }
    // Stage 2: add new vehicles
    int vehicle_num = SE_GetNumberOfObjects();
    for (int i = 0; i < vehicle_num; i++) {
        int id = SE_GetId(i);
        if (vehicles.count(id) == 0) {
            auto *v = new struct Vehicle;
            v->id = SE_GetId(i);
            v->name = SE_GetObjectName(v->id);
            v->controller = nullptr;
            vehicles.insert(std::pair<int, Vehicle *>(id, v));
        }
    }

    for (auto i: vehicles) {
        if (i.second->controller) {
            i.second->controller->step(actions[i.first]);
        }
    }
    SE_StepDT(dt);

    // Check collision state between each entity pairs, and return each collision pair
    dict collisions;
    int vehicleNums = SE_GetNumberOfObjects();
    for (int i = 0; i < vehicleNums; ++i) {
        list collisionList;
        int collisionNums = SE_GetObjectNumberOfCollisions(SE_GetId(i));
        for (int j = 0; j < collisionNums; ++j) {
            int collisionId = SE_GetObjectCollision(SE_GetId(i), j);
            collisionList.append(collisionId);
        }
        collisions[i] = collisionList;
    }

    // Get OSI
    uint8_t serializedGroundTruth[OSI_SERIALIZE_BUFFER_SIZE], serializedSensorData[OSI_SERIALIZE_BUFFER_SIZE]; // TODO
    osi_groundTruth = (osi3::GroundTruth *) SE_GetOSIGroundTruthRaw();
    osi_sensorData = (osi3::SensorData *) SE_GetOSISensorDataRaw();
    size_t gl = osi_groundTruth->ByteSizeLong();
    size_t sl = osi_sensorData->ByteSizeLong();
    osi_groundTruth->SerializeWithCachedSizesToArray(serializedGroundTruth);
    osi_sensorData->SerializeWithCachedSizesToArray(serializedSensorData);

    // package extra info
    dict info;
    info["t"] = SE_GetSimulationTime();
    info["groundTruth"] = boost::python::handle<>(
            PyMemoryView_FromMemory((char *) serializedGroundTruth, gl, PyBUF_READ));
    info["sensorData"] = boost::python::handle<>(
            PyMemoryView_FromMemory((char *) serializedSensorData, sl, PyBUF_READ));

    return make_tuple(Utils::getCurrentObs(), collisions, SE_GetQuitFlag(), info);
}

dict Lass::vehicleDict() {
    dict vdict;
    for (const auto &i: vehicles) {
        vdict[i.first] = i.second->name;
    }
    return vdict;
}

BOOST_PYTHON_MODULE (Lass) {
    class_<Lass>("Lass", init<const char *, int>())
            .def("observe", &Lass::observe)
            .def("perceive", &Lass::perceive)
            .def("step", &Lass::step)
            .def("vehicleDict", &Lass::vehicleDict);

    class_<SE_ScenarioObjectState>("ObjectState")
            .def_readonly("id", &SE_ScenarioObjectState::id)
            .def_readonly("x", &SE_ScenarioObjectState::x)
            .def_readonly("y", &SE_ScenarioObjectState::y)
            .def_readonly("z", &SE_ScenarioObjectState::z)
            .def_readonly("h", &SE_ScenarioObjectState::h)
            .def_readonly("p", &SE_ScenarioObjectState::p)
            .def_readonly("r", &SE_ScenarioObjectState::r)
            .def_readonly("s", &SE_ScenarioObjectState::s)
            .def_readonly("t", &SE_ScenarioObjectState::t)
            .def_readonly("speed", &SE_ScenarioObjectState::speed)
            .def_readonly("road_id", &SE_ScenarioObjectState::roadId)
            .def_readonly("lane_id", &SE_ScenarioObjectState::laneId)
            .def_readonly("lane_offset", &SE_ScenarioObjectState::laneOffset)
            .def_readonly("junction_id", &SE_ScenarioObjectState::junctionId)
            .def_readonly("center_offset_x", &SE_ScenarioObjectState::centerOffsetX)
            .def_readonly("center_offset_y", &SE_ScenarioObjectState::centerOffsetY)
            .def_readonly("center_offset_z", &SE_ScenarioObjectState::centerOffsetZ)
            .def_readonly("width", &SE_ScenarioObjectState::width)
            .def_readonly("length", &SE_ScenarioObjectState::length)
            .def_readonly("height", &SE_ScenarioObjectState::height);

    class_<SE_RoadInfo>("RoadInfo")
            .def_readonly("global_pos_x", &SE_RoadInfo::global_pos_x)
            .def_readonly("global_pos_y", &SE_RoadInfo::global_pos_y)
            .def_readonly("global_pos_z", &SE_RoadInfo::global_pos_z)
            .def_readonly("local_pos_x", &SE_RoadInfo::local_pos_x)
            .def_readonly("local_pos_y", &SE_RoadInfo::local_pos_y)
            .def_readonly("local_pos_z", &SE_RoadInfo::local_pos_z)
            .def_readonly("road_heading", &SE_RoadInfo::road_heading)
            .def_readonly("road_pitch", &SE_RoadInfo::road_pitch)
            .def_readonly("road_roll", &SE_RoadInfo::road_roll)
            .def_readonly("angle", &SE_RoadInfo::angle)
            .def_readonly("curvature", &SE_RoadInfo::curvature)
            .def_readonly("speed_limit", &SE_RoadInfo::speed_limit)
            .def_readonly("road_id", &SE_RoadInfo::roadId)
            .def_readonly("junction_id", &SE_RoadInfo::junctionId)
            .def_readonly("lane_id", &SE_RoadInfo::laneId)
            .def_readonly("lane_offset", &SE_RoadInfo::laneOffset)
            .def_readonly("s", &SE_RoadInfo::s)
            .def_readonly("t", &SE_RoadInfo::t);
}
