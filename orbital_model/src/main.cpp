#include <iostream>

#include "dynamics.hpp"



int main(int argc, char** argv) 
{

    /*
    Example of a sampled orbit in LEO:

        Orbital elements
            semi-major axis: 6.857865478956617e6
            eccentricity: 0.15046867207790665
            inclination: 4.743515029768906
            RAAN: 2.138912376984259
            ω (arg of perigee): 1.212845856643838
            true anomaly: 2.4479238907006007

        Cartesian state in ECI frame (x, y, z, xdot, ydot, zdot)
            [3.334844318855498e6, -4.942012834667281e6, 4.880789465113677e6, -1899.058992909672, 3296.6336870881437, 5555.923225110148])
    
    */

    std::vector<double> initial_state = {3.334844318855498e6, -4.942012834667281e6, 4.880789465113677e6, -1899.058992909672, 3296.6336870881437, 5555.923225110148};
    double initial_mass = 750;


    Spacecraft spacecraft(initial_mass, initial_state);


    std::cout << spacecraft.GetState() << std::endl;

    std::vector<double> u = {0.0, 0.0, 0.0};
    double dt = 1.0;
    
    spacecraft.Advance(u, dt);

    std::cout << spacecraft.GetState() << std::endl;


}