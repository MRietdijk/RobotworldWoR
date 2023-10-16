#ifndef COMPAS_HPP__
#define COMPAS_HPP__

#include "AbstractSensor.hpp"
#include "Robot.hpp"

namespace Model {

    class Compas : public AbstractSensor
    {
    public:
        Compas(Robot& robot, const double stdev);
        std::shared_ptr< AbstractStimulus > getStimulus() const override;
        std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
        ~Compas();
    private:
        const double stdev;
    };
    
}

#endif // COMPAS_HPP__