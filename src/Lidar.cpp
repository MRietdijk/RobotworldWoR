#include "Lidar.hpp"
#include <random>
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"
#include <vector>
#include "DistanceStimuli.hpp"
#include "DistancePercepts.hpp"
#include <math.h>
#include "Wall.hpp"
#include <memory>

namespace Model{
    Lidar::Lidar(Robot& robot, const double stdev, const uint8_t beams): AbstractSensor(robot), stdev(stdev), beams(beams)
    {

    }

    Lidar::~Lidar()
    {
    }

    std::shared_ptr< AbstractStimulus > Lidar::getStimulus() const {
        Robot* robot = dynamic_cast<Robot*>(agent);
        wxPoint robotLocation;
        
        if (robot) {
            robotLocation = robot->getPosition();
        }

        return this->getStimulus(robotLocation, beams, stdev);
    }

    std::shared_ptr< AbstractPercept > Lidar::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const {
        Robot* robot = dynamic_cast<Robot*>(agent);
        wxPoint robotLocation;
        if (robot) {
            robotLocation = robot->getPosition();
        }
    
        return this->getPerceptFor(anAbstractStimulus, robotLocation);
    }

    /* static */ std::shared_ptr< AbstractPercept > Lidar::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus, const wxPoint& location) {
        PointCloud pointcloud;
        DistanceStimuli* distanceStimuli = dynamic_cast<DistanceStimuli*>(anAbstractStimulus.get());

        for (DistanceStimulus stimulus : distanceStimuli->stimuli) {
            wxPoint endpoint{	static_cast< int >( location.x + std::cos( stimulus.angle)*stimulus.distance),
                            static_cast< int >( location.y + std::sin( stimulus.angle)*stimulus.distance)};
            DistancePercept percept(endpoint);
            pointcloud.push_back(percept);
        }

        return std::make_shared<DistancePercepts>(pointcloud);
    }

    /* static */ std::shared_ptr< AbstractStimulus > Lidar::getStimulus(const wxPoint& location, const uint8_t beams, const double stdev) {
        Stimuli stimuli;

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> noise{0, stdev};

        std::vector<std::shared_ptr<Wall>> walls = RobotWorld::getRobotWorld().getWalls();
        const double radialsBetweenBeams = (M_PI * 2) / beams;
        const uint16_t maxLenghtBeam = 1024;

        for (uint8_t i = 0; i < beams; ++i) {
            double distance = noDistance;
            double angle = radialsBetweenBeams * i;
            for (std::shared_ptr<Wall> wall : walls) {
                wxPoint laserEndpoint{static_cast<int>(location.x + std::cos( angle) * maxLenghtBeam + noise(gen)) ,
                                static_cast<int>(location.y + std::sin( angle) * maxLenghtBeam + noise(gen))};

                wxPoint interSection = Utils::Shape2DUtils::getIntersection(wall->getPoint1(), wall->getPoint2(), location, laserEndpoint);

                if (interSection != wxDefaultPosition) {
                    double nextDistance = Utils::Shape2DUtils::distance(location, interSection);
                    if (distance == noDistance || nextDistance < distance) {
                        distance = nextDistance;
                    }
                }
            }

            if (distance != noDistance) {
                DistanceStimulus stimulus(angle, distance);
                stimuli.push_back(stimulus);
            }
        }

        return std::make_shared<DistanceStimuli>(stimuli);
    }
}
