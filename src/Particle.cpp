#include "Particle.hpp"

#include "RobotWorld.hpp"
#include <cstdlib>
#include "DistancePercepts.hpp"
#include "Lidar.hpp"
#include "Shape2DUtils.hpp"

Particle::Particle() : x(std::rand() % 900), y(std::rand() % 900), weight(0)
{
}

Particle::Particle(uint16_t x, uint16_t y) : x(x), y(y), weight(0)
{
}

Particle::Particle(const Particle& other) : x(other.x), y(other.y), weight(other.weight)
{
}

Particle::~Particle()
{
}

void Particle::draw(wxDC& dc) {
  dc.SetBrush(*wxGREEN_BRUSH);
  dc.SetPen( wxPen(  "GREEN", 2, wxPENSTYLE_SOLID));
  if (weight >= 25000) {
    dc.SetBrush(*wxRED_BRUSH);
    dc.SetPen( wxPen(  "RED", 2, wxPENSTYLE_SOLID));  
  }
  dc.DrawCircle(x, y, 3);
}

void Particle::updateWeight(particleWeightType robotWeight) {
  wxPoint currPos(x, y);
  int32_t tempWeight = 0;

  std::shared_ptr< Model::AbstractPercept > percepts = Model::Lidar::getPerceptFor(Model::Lidar::getStimulus(currPos, 180, 10), currPos);
  const Model::AbstractPercept& tempPercepts{*percepts.get()};
  if (typeid(tempPercepts) == typeid(Model::DistancePercepts)) {
    Model::DistancePercepts* distancePercepts = dynamic_cast<Model::DistancePercepts*>(percepts.get());
    for (Model::DistancePercept& distancePercept : distancePercepts->pointCloud) {
      tempWeight += Utils::Shape2DUtils::distance(currPos, distancePercept.point);
    }
  }

  tempWeight -= robotWeight;
  weight = std::abs(tempWeight);
  std::cout << "Weight: " << weight << std::endl;
}

bool Particle::operator<(const Particle& other) const {
  return weight < other.weight;
}

std::string Particle::to_string() {
    return "(X: " + std::to_string(x) + ", Y: " + std::to_string(y) + " Weight: " + std::to_string(weight) + ")";
}

std::ostream& operator<<(std::ostream &strm, Particle &p) {
  return strm << p.to_string();
}

particleWeightType Particle::getWeight() const {
  return this->weight;
}

void Particle::setWeight(particleWeightType weight) {
  this->weight = weight;
}

uint16_t Particle::getX() const {
  return this->x;
}

uint16_t Particle::getY() const {
  return this->y;
}