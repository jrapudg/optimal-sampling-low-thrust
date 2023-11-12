#ifndef STATE_HPP
#define STATE_HPP 

#include <vector>
#include <iostream>
#include <cmath>



// Constants for minimum and maximum altitudes (in kilometers) for LEO, MEO, and GEO
#define LEO_MIN 160.0;  // Minimum altitude for LEO
#define LEO_MAX 2000.0; // Maximum altitude for LEO

#define MEO_MIN 2000.0; // Minimum altitude for MEO
#define MEO_MAX 35786.0; // Maximum altitude for MEO

#define GEO_MIN 35786.0; // Minimum altitude for GEO
#define GEO_MAX 42164.0; // Maximum altitude for GEO




class State 
{

public:

    // Constructors 
    State(std::vector<double>& initial_state) : data(initial_state), size(initial_state.size()) {};
    // Constructor initializing the vector with a size
    State(size_t size) : data(size, 0.0), size(size) {};
    // Copy constructor
    State(const State& other) : data(other.data), size(other.size) {};


    // Overload [] operator for element access
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    // Overload * operator for scalar multiplication
    State operator*(double scalar);

    // Overload + operator for element-wise addition
    State operator+(const State& other) const;

    // Overload << to output the state
    friend std::ostream& operator<<(std::ostream& os, const State& state);


    size_t GetSize() const;


private:
    std::vector<double> data;
    std::size_t size;

};




class State4D_ECI : public State 
{
public:
    // Constructor initializing with a 4D vector
    State4D_ECI(std::vector<double>& initial_state) : State(initial_state) {
        if (initial_state.size() != 4) {
            throw std::runtime_error("State4D_ECI must have a 4D initial state.");
        }
    }

    // Constructor initializing with zeros
    State4D_ECI() : State(4) {}
};

class State6D : public State 
{
public:
    // Constructor initializing with a 4D vector
    State6D(std::vector<double>& initial_state) : State(initial_state) {
        if (initial_state.size() != 6) {
            throw std::runtime_error("State6D must have a 6D initial state.");
        }
    }

    // Constructor initializing with zeros
    State6D() : State(6) {}
};


// TODO 6D ECI related checks (reasonable values for position)

class State6D_ECI : public State 
{
public:
    // Constructor initializing with a 4D vector
    State6D_ECI(std::vector<double>& initial_state) : State(initial_state) {
        if (initial_state.size() != 6) {
            throw std::runtime_error("State6D_ECI must have a 6D initial state.");
        }
    }

    // Constructor initializing with zeros
    State6D_ECI() : State(6) {}
};




class State6D_OE : public State 
{
public:
    // Constructor initializing with a 4D vector
    State6D_OE(std::vector<double>& initial_state) : State(initial_state) {
        if (initial_state.size() != 6) {
            throw std::runtime_error("State6D_OE must have a 6D initial state.");
        }

        /*
            The vector must contain in order:
            1. a, Semi-major axis [m]
            2. e, Eccentricity [dimensionless]
            3. i, Inclination [rad]
            4. w, Right Ascension of the Ascending Node (RAAN) [rad]
            5. w, Argument of Perigee [ramd]
            6. t, Mean anomaly [rad]
        */

        double a = initial_state[0]; // Semi-major axis [m]
        double e = initial_state[1]; // Eccentricity [dimensionless]
        double i = initial_state[2]; // Inclination [rad]
        double RAAN = initial_state[3]; // Right Ascension of the Ascending Node (RAAN) [rad]
        double w = initial_state[4]; // Argument of Perigee [rad]
        double t = initial_state[5]; // Mean anomaly [rad]

        if (a <= 0.0) {
            throw std::runtime_error("Semi-major axis (a) must be greater than 0.");
        }
        if (e < 0.0 || e >= 1.0) {
            throw std::runtime_error("Eccentricity (e) must be in the range [0, 1).");
        }
        if (i < 0.0 || i >= 2 * M_PI) {
            throw std::runtime_error("Inclination (i) must be in the range [0, 2π).");
        }
        if (RAAN < 0.0 || RAAN >= 2 * M_PI) {
            throw std::runtime_error("Right Ascension of the Ascending Node (RAAN) must be in the range [0, 2π).");
        }
        if (w < 0.0 || w >= 2 * M_PI) {
            throw std::runtime_error("Argument of Perigee (w) must be in the range [0, 2π).");
        }
        if (t < 0.0 || t >= 2 * M_PI) {
            throw std::runtime_error("Mean anomaly (M) must be in the range [0, 2π).");
        }
    }

};



class Control : public State 
{
public:

    Control(std::vector<double>& initial_state) : State(initial_state) {
        if (initial_state.size() != 6) {
            throw std::runtime_error("Control2D must have a 2D initial state.");
        }
    }

    Control(size_t size) : State(size) {}

};




class Control2D : public Control
{
public:
    // Constructor initializing with a 2D vector
    Control2D(std::vector<double>& initial_state) : Control(initial_state) {
        if (initial_state.size() != 2) {
            throw std::runtime_error("Control2D must have a 2D initial state.");
        }
    }

    // Constructor initializing with zeros
    Control2D() : Control(2) {}

};


class Control3D : public Control 
{
public:
    // Constructor initializing with a 3D vector
    Control3D(std::vector<double>& initial_state) : Control(initial_state) {
        if (initial_state.size() != 3) {
            throw std::runtime_error("Control3D must have a 3D initial state.");
        }
    }

    // Constructor initializing with zeros
    Control3D() : Control(3) {}

};



#endif 