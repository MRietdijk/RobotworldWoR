#include <stdint.h>
#include <string>

class Particle
{
public:
    Particle(uint16_t maxX, uint16_t maxY);
    ~Particle();
    std::string to_string();
private:
    friend std::ostream& operator<<(std::ostream&, Particle&);
    uint16_t x;
    uint16_t y;
};