#ifndef PARTICLE_H__
#define PARTICLE_H__


#include <stdint.h>
#include <string>
#include "Widgets.hpp"

typedef uint32_t particleWeightType;
class Particle
{
public:
    Particle(uint16_t maxX, uint16_t maxY);
    Particle(const Particle& other);
    ~Particle();
    void draw(wxDC& dc);
    void updateWeight(particleWeightType robotWeight);
    particleWeightType getWeight() const;
    std::string to_string();
    bool operator<(const Particle& other) const;
    void setWeight(particleWeightType weight);
private:
    friend std::ostream& operator<<(std::ostream&, Particle&);
    uint16_t x;
    uint16_t y;
    particleWeightType weight;
};

#endif