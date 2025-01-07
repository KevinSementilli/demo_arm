#ifndef DEMO_ARM_SERVO_HPP
#define DEMO_ARM_SERVO_HPP

#include <string>
#include <cmath>

class Servo {

public:
    
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    // double rads_per_count = 0;

    Servo() = default;

    Servo(const std::string &actuator_name) {

        setup(actuator_name);
    }

    void setup(const std::string &actuator_name)
    {
        name = actuator_name;
        // rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calc_enc_angle()
    {
      return enc;
    }


};


#endif