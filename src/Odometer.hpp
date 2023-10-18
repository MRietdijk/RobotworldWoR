#ifndef ODOMETER_HPP__
#define ODOMETER_HPP__

#include "Robot.hpp"
#include "AbstractSensor.hpp"
#include "RotationPercept.hpp"

namespace Model {

    class Odometer: public AbstractSensor
    {
    public:
        Odometer(Robot& robot, const double stdev);
        std::shared_ptr< AbstractStimulus > getStimulus() const override;
        std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
        ~Odometer();
    private:
        const double stdev;
    };
}




#endif // ODOMETER_HPP__