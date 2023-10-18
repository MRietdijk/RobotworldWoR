#ifndef ROTATIONPERCEPT_HPP__
#define ROTATIONPERCEPT_HPP__

#include "RotationStimulus.hpp"

namespace Model {

    class RotationPercept : public AbstractPercept
    {
    public:
        RotationPercept() : rotations(0) {};
        RotationPercept(double rotations): rotations(rotations) {};
        double rotations;

        virtual std::string asString() const override
        {
            return "RotationPercept: " + std::to_string(rotations);
        }

        virtual std::string asDebugString() const override
        {
            return asString();
        }
    };

}


#endif // ROTATIONPERCEPT_HPP__