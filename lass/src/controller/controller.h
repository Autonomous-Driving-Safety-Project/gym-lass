#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <esmini/esminiLib.hpp>
#include <boost/python.hpp>

namespace Controller {
    class Controller {
    public:
        Controller(int id, float dt) : objectId(id), dt(dt) {};
        virtual ~Controller() = default;
        virtual void step(boost::python::object action) = 0;
    protected:
        int objectId;
        float dt;
    };
}

#endif
