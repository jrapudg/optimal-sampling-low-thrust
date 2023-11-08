#include <cmath>
#include <stdexcept>
#include <iostream>

#include "dynamics.hpp"



double& State::operator[](size_t index) {
    return data[index];
}

const double& State::operator[](size_t index) const {
    return data[index];
}

State State::operator*(double scalar) {
        State result(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result[i] = data[i] * scalar;
        }
        return result;
    }



State State::operator+(const State& other) const {
    if (data.size() != other.data.size()) {
        throw std::runtime_error("Vector sizes do not match for addition.");
    }
    State result(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        result[i] = data[i] + other[i];
    }
    return result;
}


// Define the << operator outside the class
std::ostream& operator<<(std::ostream& os, const State& state) {
    os << "[";
    for (size_t i = 0; i < state.data.size(); ++i) {
        os << state.data[i];
        if (i < state.data.size() - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}



// Central body dynamics
State Spacecraft::Dynamics(const State& x, const std::vector<double>& u) {

    State next_state(6);

    // Velocity
    for (int i = 0; i < 3; i++) {
        next_state[i] = x[i + 3]; 
    }

    // Acceleration
    double norm_r = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    for (int i = 3; i < 6; i++) {
        next_state[i] = - (MU / std::pow(norm_r, 3)) * x[i] + (1.0 / mass) * u[i - 3];
    }
    return next_state;
}






void Spacecraft::RK4(const State& state, const std::vector<double>& u, double dt, State& next_state) {
    State k1 = Dynamics(state, u);
    State k2 = Dynamics(state + k1 * 0.5, u);
    State k3 = Dynamics(state + k2 * 0.5, u);
    State k4 = Dynamics(state + k3, u);

    next_state = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0); 
    //current_state = next_state;
}


void Spacecraft::Advance(const std::vector<double>& u, double dt)
{
    State next_state(6);

    RK4(current_state, u, dt, next_state);
    current_state = next_state;

}


const State& Spacecraft::GetState() const
{
    return current_state;
}