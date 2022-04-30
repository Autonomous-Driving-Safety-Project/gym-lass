//
// Created by violin on 2022/2/24.
//

#include "naive.h"
#include <boost/python.hpp>

namespace Controller {
    Naive::Naive(int id, float dt) : Controller(id, dt) {
//        printf("Index:%d Id:%d Name:%s\n", index, this->objectId, SE_GetObjectName(this->objectId));
        SE_ScenarioObjectState objState;
        SE_GetObjectState(objectId, &objState);
        length = objState.length;
//        printf("Object State: %f %f %f %f %f\n", objState.x, objState.y, objState.h, objState.length,objState.speed);
        vehicleHandle = SE_SimpleVehicleCreate(objState.x, objState.y, objState.h, length, objState.speed);
    }

    Naive::Naive(int id, float dt, float length) : Controller(id, dt), length(length) {
//        printf("Index:%d Id:%d Name:%s\n", index, this->objectId, SE_GetObjectName(this->objectId));
        SE_ScenarioObjectState objState;
        SE_GetObjectState(objectId, &objState);
//        printf("Object State: %f %f %f %f %f\n", objState.x, objState.y, objState.h, objState.length,objState.speed);
        vehicleHandle = SE_SimpleVehicleCreate(objState.x, objState.y, objState.h, length, objState.speed);

//        SE_SimpleVehicleState state;
//        SE_SimpleVehicleGetState(vehicleHandle, &(state));
//        printf("State: %f %f %f %f %p\n", state.x, state.y, state.h, state.speed, vehicleHandle);
    }

    Naive::~Naive() {
        SE_SimpleVehicleDelete(this->vehicleHandle);
    }

    void Naive::step(boost::python::object action) {
        double throttle = boost::python::extract<double>(action[0]);
        double steer = boost::python::extract<double>(action[1]);
        SE_SimpleVehicleState state;
//        SE_SimpleVehicleGetState(vehicleHandle, &(state));
//        printf("State: %f %f %f %f %p\n", state.x, state.y, state.h, state.speed, vehicleHandle);
        if (SE_GetSimulationTime() > 0)
        {
            SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steer);
        }
//        printf("Action: %f %f %f\n", dt, throttle, steer);
        SE_SimpleVehicleGetState(vehicleHandle, &(state));
//        printf("State: %f %f %f %f\n", state.x, state.y, state.h, state.speed);
        SE_ReportObjectPosXYH(objectId, 0, state.x, state.y, state.h, state.speed);
    }
}