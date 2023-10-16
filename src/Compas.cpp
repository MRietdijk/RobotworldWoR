#include "Compas.hpp"

#include "AngleStimulus.hpp"
#include "AnglePercept.hpp"
#include "Shape2DUtils.hpp"
#include <random>

namespace Model {
    Compas::Compas(Robot& robot, const double stdev): AbstractSensor(robot), stdev(stdev)
    {
    }

    Compas::~Compas()
    {
    }

    std::shared_ptr< AbstractStimulus > Compas::getStimulus() const {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> noise{0, stdev};

        Robot* robot = dynamic_cast<Robot*>(agent);
        
        if (robot) {
            BoundedVector front = robot->getFront();

            return std::make_shared<AngleStimulus>(front);
        }
    }

    std::shared_ptr< AbstractPercept > Compas::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const {
        AngleStimulus* angleStimulus = dynamic_cast<AngleStimulus*>(anAbstractStimulus.get());

        double angle = Utils::Shape2DUtils::getAngle( angleStimulus->vector) + 0.5 * Utils::PI;

        return std::make_shared<AnglePercept>(angle);
    }
}