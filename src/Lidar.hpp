#ifndef LIDAR_H__
#define LIDAR_H__

#include "AbstractSensor.hpp"
#include "stdint.h"
#include "Robot.hpp"

namespace Model {

    class Lidar : public AbstractSensor
    {
    public:
        explicit Lidar(Robot& robot, const double stdev, const uint8_t beams);
        std::shared_ptr< AbstractStimulus > getStimulus() const override;
        std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
        static std::shared_ptr< AbstractStimulus > getStimulus(const wxPoint& location, const uint8_t beams, const double stdev);
        static std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus, const wxPoint& location);
        ~Lidar();
    private:
        const double stdev;
        const uint8_t beams;
    };
}



#endif