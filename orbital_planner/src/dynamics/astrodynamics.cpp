#include "astrodynamics.hpp"

namespace Astrodynamics
{

bool CheckOrbitalElements(const StateOE& orbital_elements)
{
    /*
        The vector must contain in order:
        1. a, Semi-major axis [m]
        2. e, Eccentricity [dimensionless]
        3. i, Inclination [rad]
        4. w, Right Ascension of the Ascending Node (RAAN) [rad]
        5. w, Argument of Perigee [ramd]
        6. t, Mean anomaly [rad]
    */

    double a = orbital_elements[0]; // Semi-major axis [m]
    double e = orbital_elements[1]; // Eccentricity [dimensionless]
    double i = orbital_elements[2]; // Inclination [rad]
    double RAAN = orbital_elements[3]; // Right Ascension of the Ascending Node (RAAN) [rad]
    double w = orbital_elements[4]; // Argument of Perigee [rad]
    double t = orbital_elements[5]; // Mean anomaly [rad]

    if (a <= 0.0) {
        throw std::runtime_error("Semi-major axis (a) must be greater than 0.");
        //return false;
    }
    if (e < 0.0 || e >= 1.0) {
        throw std::runtime_error("Eccentricity (e) must be in the range [0, 1).");
        //return false;
    }
    if (i < 0.0 || i >= 2 * M_PI) {
        throw std::runtime_error("Inclination (i) must be in the range [0, 2π).");
        //return false;
    }
    if (RAAN < 0.0 || RAAN >= 2 * M_PI) {
        throw std::runtime_error("Right Ascension of the Ascending Node (RAAN) must be in the range [0, 2π).");
        //return false;
    }
    if (w < 0.0 || w >= 2 * M_PI) {
        throw std::runtime_error("Argument of Perigee (w) must be in the range [0, 2π).");
        //return false;
    }
    if (t < 0.0 || t >= 2 * M_PI) {
        throw std::runtime_error("Mean anomaly (M) must be in the range [0, 2π).");
        //return false;
    }

    return true;
}



Vector toEigen(std::vector<double> v)
{
    int size = v.size();
    Vector vec(size);
    for (size_t i = 0; i < size; ++i) {
        vec[i] = v[i];
    }
    return vec;
}



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
    4. RAAN, Right Ascension of the Ascending Node (RAAN) [rad]
    5. w, Argument of Perigee [ramd]
    6. t, Mean anomaly [rad]
*/

void OrbitalSampler::SampleOrbit(const StateOE& orb_elem_min, const StateOE& orb_elem_max, State &sampled_eci_state)
{
    
    StateOE orbital_elements;

    for (int i = 0; i < 6; ++i)
    {
        orbital_elements[i] = Sample(orb_elem_min[i], orb_elem_max[i]);
    }

    sampled_eci_state = OE2ECI(orbital_elements);

}



void OrbitalSampler::SampleOrbit(std::string region, State &sampled_eci_state)
{
    const double earth_radius_km = 6371.0;
    double min_altitude_km, max_altitude_km;
    
    if (region == "LEO") {
        min_altitude_km = LEO_MIN;
        max_altitude_km = LEO_MAX;
    } else if (region == "MEO") {
        min_altitude_km = MEO_MIN;
        max_altitude_km = MEO_MAX;
    } else if (region == "GEO") {
        min_altitude_km = GEO_MIN;
        max_altitude_km = GEO_MAX;
    } else {
        throw std::invalid_argument("Unknown orbit region specified.");
    }

    // Convert altitudes to semi-major axis lengths by adding Earth's radius
    double min_sma = (earth_radius_km + min_altitude_km) * 1000.0; // convert to meters
    double max_sma = (earth_radius_km + max_altitude_km) * 1000.0; // convert to meters

    // Sample the semi-major axis within the specified range
    double a = Sample(min_sma, max_sma);
    double e =Sample(0, 1.0); 
    double i = Sample(0, M_PI); 
    double RAAN = Sample(0, 2 * M_PI); 
    double arg_peri = Sample(0, 2 * M_PI);
    double mean_anom = Sample(0, 2 * M_PI); 


    StateOE orbital_elements = {a, e, i, RAAN, arg_peri, mean_anom};

    // Convert the orbital elements to an ECI state vector
    sampled_eci_state = OE2ECI(orbital_elements);
}




void OrbitalSampler::SampleAroundOrbit(const State &mean_eci_state, const StateOE& std_oe)
{
    // 
}


// Hard-coded 
void OrbitalSampler::SampleCW(State &sampled_state, double max_dist_lvlh, double max_vel_lvlh)
{
    for (int i = 0; i < 3; ++i)
    {
        sampled_state[i] = Sample(- max_dist_lvlh, max_dist_lvlh);
    }

    for (int i = 3; i < 6; ++i)
    {
        sampled_state[i] = Sample(- max_vel_lvlh, max_vel_lvlh);
    }

}



void CentralBody(const State& x, const Control& u, State& next_state)
{
    // Velocity
    for (int i = 0; i < 3; i++) {
        next_state[i] = x[i + 3]; 
    }

    // Acceleration
    double norm_r = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int i = 3; i < 6; i++) {
        next_state[i] = - (MU / std::pow(norm_r, 3)) * x[i] + u[i - 3] / DEFAULT_MASS; 
    }
}



// Assumes circular orbit and distance between two spacecraft is << distance to central body
void ClohessyWiltshire(const State &x, const Control &u, State &next_state)
{
    // Velocity
    next_state.segment(0, 3) = x.segment(3, 3);

    // Assume for now LEO - should be a parameter
    double sma = LEO_MAX;
    const double n = std::sqrt(MU / std::pow(sma, 3));

    // Acceleration 
    next_state[3] = 3 * n * n * x[0] + 2 * n * x[4] + u[0] / DEFAULT_MASS; 
    next_state[4] = -2* n * x[3] + u[1] / DEFAULT_MASS;
    next_state[5] = - n * n * x[2] + u[2] / DEFAULT_MASS;
}


void GetClohessyWiltshireMatrices(MatrixA& A, MatrixB& B, double m, double sma) 
{

    const double n = std::sqrt(MU / std::pow(sma, 3));

    // Define matrix A
    A << 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1,
         3*n*n, 0, 0, 0, 2*n, 0,
         0, 0, 0, -2*n, 0, 0,
         0, 0, -n*n, 0, 0, 0;

    // Define matrix B
    B << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        1, 0, 0,
        0, 1, 0, 
        0, 0, 1;

    B /= DEFAULT_MASS;

}



void CentralBodyScaled(const State &state, const Control &control, State &next_state)
{
    // Update next_state based on scaled calculations
}

void CR3BP(const State &state, const Control &control, State &state_dot)
{
    // Update state_dot for CR3BP
}

State OE2ECI(const StateOE& orbital_elements)
{
    // TODO
    return State();
}

StateOE ECI2OE(const State& eci_state)
{
    // TODO
    return State();
}


void Print(const State& s,  std::string name) 
{
    std::cout << name << ": [";
    for (int i = 0; i < s.size(); ++i) {
        std::cout << s[i];
        if (i < s.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

void Print(const Control& c,  std::string name) 
{
    std::cout <<  name << ": [";
    for (int i = 0; i < c.size(); ++i) {
        std::cout << c[i];
        if (i < c.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}




void PrintOE(const StateOE& oe)
{
    std::cout << "Orbital elements: " << std::endl;
    std::cout << "a, Semi-major axis [m]: " << oe[0] << std::endl;
    std::cout << "e, Eccentricity [dimensionless]: " << oe[1] << std::endl;
    std::cout << "i, Inclination [rad]: " << oe[2] << std::endl;
    std::cout << "RAAN, Right Ascension of the Ascending Node (RAAN) [rad]: " << oe[3] << std::endl;
    std::cout << "w, Argument of Perigee [rad]: " << oe[4] << std::endl;
    std::cout << "t, Mean anomaly [rad]: " << oe[5] << std::endl;
}







}





