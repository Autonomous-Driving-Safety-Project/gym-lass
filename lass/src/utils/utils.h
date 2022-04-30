//
// Created by violin on 2022/02/22
//

#ifndef UTILS_H
#define UTILS_H

#include <boost/geometry.hpp>
#include <boost/python.hpp>
#include "esmini/esminiLib.hpp"

typedef boost::geometry::model::d2::point_xy<double> point_t;
typedef boost::geometry::model::ring<point_t, true, false> box_t;

namespace Utils {
    int gap(int egoIndex, double *gaps, int maxObj);
//    boost::python::dict getCurrentObs(int objectId);
    SE_ScenarioObjectState getCurrentObs(int objectId);
    boost::python::list getCurrentObs();
}

#endif // UTILS_H