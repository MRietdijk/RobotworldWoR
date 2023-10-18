#ifndef ROTATIONSTIMULUS_HPP__
#define ROTATIONSTIMULUS_HPP__

#include "AbstractSensor.hpp"

namespace Model {

    class RotationStimulus : public AbstractStimulus
    {
    public:
        RotationStimulus(const double rotations): rotations(rotations) {};

        virtual std::string asString() const override
        {
            return "RotationStimulus: " + std::to_string(rotations);
        }

        virtual std::string asDebugString() const override
        {
            return asString();
        }
        const double rotations;
    };
}


#endif // ROTATIONSTIMULUS_HPP__