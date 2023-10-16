#ifndef ANGLEPERCEPT_HPP__
#define ANGLEPERCEPT_HPP__

#include "AngleStimulus.hpp"

namespace Model {

    class AnglePercept: public AbstractPercept
    {
    public:
        AnglePercept(double angle) : angle(angle) {};
        double angle;

        virtual std::string asString() const
        {
            return "AnglePercept: " + std::to_string(angle);
        }

        virtual std::string asDebugString() const
        {
            return asString();
        }
    };
}


#endif // ANGLEPERCEPT_HPP__