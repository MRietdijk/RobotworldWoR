#include "Particle.h"
#include <cstdlib>

Particle::Particle(uint16_t maxX, uint16_t maxY) : x(std::rand() % maxX), y(std::rand() % maxY)
{
}

Particle::~Particle()
{
}

std::string Particle::to_string() {
    return "(X: " + std::to_string(x) + ", Y: " + std::to_string(y) + ")";
}

std::ostream& operator<<(std::ostream &strm, Particle &p) {
  return strm << p.to_string();
}