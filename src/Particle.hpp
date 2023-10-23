#ifndef PARTICLE_H__
#define PARTICLE_H__


#include <stdint.h>
#include <string>
#include "Widgets.hpp"

typedef double particleWeightType;
class Particle
{
public:
    Particle();
    Particle(uint16_t x, uint16_t y);
    Particle(const Particle& other);
    ~Particle();
    void draw(wxDC& dc);
    void updateWeight(std::vector<particleWeightType> robotWeight);
    void updatePosition(int8_t x, int8_t y);
    particleWeightType getWeight() const;
    std::string to_string();
    bool operator<(const Particle& other) const;
    void setWeight(particleWeightType weight);
    uint16_t getX() const;
    uint16_t getY() const;
private:
    friend std::ostream& operator<<(std::ostream&, Particle&);
    uint16_t x;
    uint16_t y;
    particleWeightType weight;
};

#endif