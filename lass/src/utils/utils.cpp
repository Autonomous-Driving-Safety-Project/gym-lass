//
// Created by violin on 2022/02/22.
//

#include "utils.h"
#include "esmini/esminiLib.hpp"

static void getBox(double x, double y, double length, double width, double h, box_t *&box)
{
    double lx = length / 2.0 * fabs(cos(h));
    double ly = length / 2.0 * fabs(sin(h));
    double wx = width / 2.0 * fabs(sin(h));
    double wy = width / 2.0 * fabs(cos(h));
    point_t p1(x + lx + wx, y + ly - wy);
    point_t p2(x - lx + wx, y - ly - wy);
    point_t p3(x - lx - wx, y - ly + wy);
    point_t p4(x + lx - wx, y + ly + wy);
    std::initializer_list<point_t> l = {p1, p2, p3, p4};
    box = new box_t(l);
}

namespace Utils
{
    SE_ScenarioObjectState getCurrentObs(int objectId) {
        SE_ScenarioObjectState state;
        SE_GetObjectState(objectId, &state);
        return state;
    }

    boost::python::list getCurrentObs()
    {
        int num = SE_GetNumberOfObjects();
        boost::python::list obs;
        for(int i=0; i<num; i++)
        {
            SE_ScenarioObjectState obs_i = getCurrentObs(SE_GetId(i));
            obs.append(obs_i);
        }
        return obs;
    }
}
