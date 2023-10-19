#include "Odometer.hpp"
#include <random>
#include "Shape2DUtils.hpp"

namespace Model {
    Odometer::Odometer(Robot& robot, const double stdev): AbstractSensor(robot), stdev(stdev)
    {
    }

    std::shared_ptr< AbstractStimulus > Odometer::getStimulus() const {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> noise{-stdev, stdev};
        Robot* robot = dynamic_cast<Robot*>(agent);
        
        if (robot) {
            double newRotations = Utils::Shape2DUtils::distance(robot->getPrevPosition(), robot->getPosition()) + noise(gen);

            return std::make_shared<RotationStimulus>(newRotations);
        }
    }
    
    
    std::shared_ptr< AbstractPercept > Odometer::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const {
        RotationStimulus* rotationStimulus = dynamic_cast<RotationStimulus*>(anAbstractStimulus.get());
        RotationPercept percept(rotationStimulus->rotations);

        return std::make_shared<RotationPercept>(percept);
    }
        

    Odometer::~Odometer()
    {
    }
}
