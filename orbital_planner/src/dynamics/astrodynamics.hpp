#ifndef ASTRODYNAMICS_HPP
#define ASTRODYNAMICS_HPP

#include <string>
#include <random>
#include <vector>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>


namespace Astrodynamics
{



typedef Eigen::VectorXd Vector;


//////// State ////////

typedef Eigen::Matrix<double, 6, 1> State;
typedef Eigen::Matrix<double, 3, 1> Control;
typedef Eigen::Matrix<double, 6, 1> StateOE;

typedef Eigen::Matrix<double, 6, 6> MatrixA;
typedef Eigen::Matrix<double, 6, 3> MatrixB;


// Astrodynamics constrants 
#define MU 3.986004418e14
#define G 6.67430e-11
#define M 5.9722e24

// Constants for minimum and maximum altitudes (in meters) for LEO, MEO, and GEO
#define LEO_MIN 160000.0  // Minimum altitude for LEO
#define LEO_MAX 2000000.0 // Maximum altitude for LEO

#define MEO_MIN 2000000.0 // Minimum altitude for MEO
#define MEO_MAX 35786000.0 // Maximum altitude for MEO

#define GEO_MIN 35786000.0 // Minimum altitude for GEO
#define GEO_MAX 42164000.0 // Maximum altitude for GEO


// Default maximum distance and velocities for unform sampling in the LVLH frame (Clohessy-Wiltshire)
#define MAX_DIST_LVLH 5.0
#define MAX_VEL_LVLH 1.5

// Assumed default mass of the spacecraft
#define DEFAULT_MASS 100.0
#define ASSUMED_SMA LEO_MAX


// Dynamics function 

void CentralBody(const State& state, const Control& control, State& state_dot);

void CentralBodyScaled(const State& state, const Control& control, State& state_dot);

void CR3BP(const State& state, const Control& control, State& state_dot);

void NonlinearRelativeKeplerianDynamics(const State& state, const Control& u, State& next_state);

void ClohessyWiltshire(const State& state, const Control& control, State& next_state);

void GetClohessyWiltshireMatrices(MatrixA& A, MatrixB& B, double m = 100.0, double sma = LEO_MAX);



// Orbit sampling

class OrbitalSampler
{
    public:

        OrbitalSampler(int seed = 0);

        void SampleOrbit(const StateOE &orb_elem_min, const StateOE &orb_elem_max, State &sampled_eci_state);

        void SampleOrbit(std::string region, State &sampled_eci_state);

        // Sample within a given delta OE around the current orbit 
        void SampleAroundOrbit(const State &mean_eci_state, const StateOE &std_oe);

        void SampleCW(State &sampled_state, double max_dist_lvlh = MAX_DIST_LVLH, double max_vel_lvlh = MAX_VEL_LVLH);


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



// Helper functions 
/*
    The vector must contain in order:
    1. a, Semi-major axis [m]
    2. e, Eccentricity [dimensionless]
    3. i, Inclination [rad]
    4. w, Right Ascension of the Ascending Node (RAAN) [rad]
    5. w, Argument of Perigee [ramd]
    6. t, Mean anomaly [rad]
*/
bool CheckOrbitalElements(const StateOE& orbital_elements);

Vector toEigen(std::vector<double> v);

State OE2ECI(const StateOE& orbital_elements);

StateOE ECI2OE(const State& eci_state);

void PrintOE(const StateOE& oe);
void Print(const State& s, std::string name = "State");
void Print(const Control& c, std::string name = "Control");



}

#endif

