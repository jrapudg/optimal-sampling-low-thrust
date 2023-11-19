
#include "astrodynamics.hpp"



namespace Astrodynamics
{


OrbitalSampler::OrbitalSampler(int seed)
{
    // From hardware
    std::random_device rd;

    if (seed == 0)
    {
        rng = std::mt19937(rd());
    } else 
    {
        rng = std::mt19937(seed);
    }

    // Uniform distribution for angles
    angle_dis = std::uniform_real_distribution<double>(0, 2 * M_PI);

    // Uniform distribution 
    uniform_dis = std::uniform_real_distribution<double>(0, 1.0);


    // Gaussian
    gaussian_dis = std::normal_distribution<double>(0, 1);
}





double OrbitalSampler::Lerp(double min, double max, double t) {
    return min + t * (max - min);
}

// Sample a value between min and max
double OrbitalSampler::Sample(double min, double max) {

    // Use linear interpolation to get a value between min and max
    return Lerp(min, max, uniform_dis(rng));
}


/*
    Orbital element vector order:
    1. a, Semi-major axis [m]
    2. e, Eccentricity [dimensionless]
    3. i, Inclination [rad]
    4. w, Right Ascension of the Ascending Node (RAAN) [rad]
    5. w, Argument of Perigee [ramd]
    6. t, Mean anomaly [rad]
*/

void OrbitalSampler::SampleOrbit(State6D_OE orb_elem_min, State6D_OE orb_elem_max, State6D_ECI &sampled_eci_state)
{
    
    std::vector<double> orbital_elements;

    for (int i = 0; i < 6; ++i)
    {
        orbital_elements.push_back(Sample(orb_elem_min[i], orb_elem_max[i]));
    }

    State6D_OE oe(orbital_elements);

    sampled_eci_state = OE2ECI(oe);

}

void OrbitalSampler::SampleOrbit(State6D_OE orb_elem_min, State6D_OE orb_elem_max, State4D_ECI &sampled_eci_state)
{

    // Reuse the 6D version
}



void OrbitalSampler::SampleOrbit(std::string region, State6D_ECI &sampled_eci_state)
{

}

void OrbitalSampler::SampleAroundOrbit(State6D_ECI current_state, State6D_ECI &sampled_eci_state)
{

}




void PlanarCentralBody(State4D_ECI x, Control2D u, State4D_ECI& next_state)
{
    // Velocity
    for (int i = 0; i < 2; i++) {
        next_state[i] = x[i + 2]; 
    }

    // Acceleration
    double norm_r = std::sqrt(x[0] * x[0] + x[1] * x[1]);
    for (int i = 2; i < 4; i++) {
        next_state[i] = - (MU / std::pow(norm_r, 3)) * x[i] + u[i - 2]; // Control input is acceleration
    }
}

void CentralBody(State6D_ECI x, Control3D u, State6D_ECI& next_state)
{
    // Velocity
    for (int i = 0; i < 3; i++) {
        next_state[i] = x[i + 3]; 
    }

    // Acceleration
    double norm_r = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int i = 3; i < 6; i++) {
        next_state[i] = - (MU / std::pow(norm_r, 3)) * x[i] + u[i - 3]; // Control input is acceleration
    }
}

// Assumes circular orbit and distance between two spacecraft is << distance to central body
void ClohessyWiltshire(State6D x, Control3D u, State6D& next_state)
{
    // Velocity
    for (int i = 0; i < 3; i++) {
        next_state[i] = x[i + 3]; 
    }

    // Assume for now LEO - should be a parameter
    double sma = LEO_MAX;
    const double n = std::sqrt(3.986004418e14 / std::pow(sma, 3));


    // Acceleration - // Control input is acceleration

    next_state[3] = 3 * n * n * x[0] + 2 * n * x[4] + u[0]; 
    next_state[4] = -2* n * x[3] + u[1];
    next_state[5] = - n * n * x[2] + u[2];

}






void PlanarCentralBodyScaled(State4D_ECI x, Control2D u, State4D_ECI& next_state)
{
    // Update next_state based on scaled calculations
}

void CentralBodyScaled(State6D_ECI state, Control3D control, State6D_ECI& next_state)
{
    // Update next_state based on scaled calculations
}

void CR3BP(State6D state, Control3D control, State6D& state_dot)
{
    // Update state_dot for CR3BP
}





State6D_ECI OE2ECI(State6D_OE orbital_elements)
{

}

State6D_OE ECI2OE(State6D_ECI eci_state)
{
    
}

}