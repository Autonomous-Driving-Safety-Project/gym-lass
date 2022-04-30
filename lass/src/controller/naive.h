//
// Created by violin on 2022/2/24.
//

#ifndef NAIVE_H
#define NAIVE_H

#include "controller.h"
#include "esmini/esminiLib.hpp"

namespace Controller
{
    class Naive : public Controller
    {
    public:
        Naive(int index, float dt);
        Naive(int index, float dt, float length);
        ~Naive() override;
        void step(boost::python::object action) override;
    private:
        void *vehicleHandle;
        float length;
    };
}

#endif //SCENARIO_NAIVE_H
