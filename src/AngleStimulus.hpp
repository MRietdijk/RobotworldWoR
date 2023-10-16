#ifndef ANGLESTIMULUS_HPP__
#define ANGLESTIMULUS_HPP__

#include "AbstractSensor.hpp"
#include "BoundedVector.hpp"

namespace Model {

    class AngleStimulus: public AbstractStimulus
    {
    public:
        AngleStimulus(BoundedVector vector) : vector(vector) {};
        BoundedVector vector;

        virtual std::string asString() const
        {
            return "AngleStimulus: " + std::to_string(vector.x) + ", " + std::to_string(vector.y);
        }

        virtual std::string asDebugString() const
        {
            return asString();
        }
    };
    
}

#endif // ANGLESTIMULUS_HPP__