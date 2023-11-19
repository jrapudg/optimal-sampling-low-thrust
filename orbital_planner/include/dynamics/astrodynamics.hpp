#include <string>

#include <random>
#include "states.hpp"


namespace Astrodynamics
{


#define MU 3.986004418e14
#define G 6.67430e-11
#define M 5.9722e24



/*
TODO 
- Docs for each function 
- Implement scaled dynamics
- Implement CR3BP
- Implement all sampler fct from the OrbitalSampler class


*/


// Orbit sampling

class OrbitalSampler
{
    public:

        OrbitalSampler(int seed = 0);

        void SampleOrbit(State6D_OE orb_elem_min, State6D_OE orb_elem_max, State4D_ECI &sampled_eci_state);

        void SampleOrbit(State6D_OE orb_elem_min, State6D_OE orb_elem_max, State6D_ECI &sampled_eci_state);

        // TODO will just use above
        void SampleOrbit(std::string region, State6D_ECI &sampled_eci_state);

        // Sample within a given delta OE around the current orbit 
        void SampleAroundOrbit(State6D_ECI &sampled_eci_state);


    private:

        std::mt19937 rng; 
        std::uniform_real_distribution<double> angle_dis;
        std::uniform_real_distribution<double> uniform_dis;
        std::normal_distribution<double> gaussian_dis;

        // Linearly interpolate between two values
        double Lerp(double min, double max, double t);

        // Sample a value between min and max
        double Sample(double min, double max);



};


// Dynamics function 

void PlanarCentralBody(State4D_ECI state, Control2D control, State4D_ECI& state_dot);

void CentralBody(State6D_ECI state, Control3D control, State6D_ECI& state_dot);

void ClohessyWiltshire(State6D x, Control3D u, State6D& next_state);

void PlanarCentralBodyScaled(State4D_ECI state, Control2D control, State4D_ECI& state_dot);

void CentralBodyScaled(State6D_ECI state, Control3D control, State6D_ECI& state_dot);

void CR3BP(State6D state, Control3D control, State6D& state_dot);




State6D_ECI OE2ECI(State6D_OE orbital_elements);

State6D_OE ECI2OE(State6D_ECI eci_state);



}