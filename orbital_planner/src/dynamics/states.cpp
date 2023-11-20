#include "states.hpp"


//////// State ////////


//// Constructors 

// Constructor w/ vector
State::State(std::vector<double>& initial_state) 
: data(initial_state), _size(initial_state.size()) {};

// Constructor initializing the vector with a size
State::State(size_t size) 
: data(size, 0.0), _size(size) {};

// Copy constructor
State::State(const State& other) 
: data(other.data), _size(other._size) {};

// Constructor with initializer list
State::State(std::initializer_list<double> initial_state) 
: data(initial_state), _size(initial_state.size()) {}

// Constructor initializing with an Eigen::VectorXd
State::State(const Eigen::VectorXd& initial_state) 
: data(EigenToVector(initial_state)) {}

void State::GetData(std::vector<double>& out)
{
    for (int i = 0; i <= _size; ++i)
    {
        out[i] = data[i];
    }
}


double& State::operator[](size_t index) {
    return data[index];
}

const double& State::operator[](size_t index) const {
    return data[index];
}

State State::operator*(double scalar) {
        State result(_size);
        for (size_t i = 0; i < _size; ++i) {
            result[i] = data[i] * scalar;
        }
        return result;
    }



State State::operator+(const State& other) const {
    if (_size != other.data.size()) {
        throw std::runtime_error("Vector sizes do not match for addition.");
    }
    State result(_size);
    for (size_t i = 0; i < _size; ++i) {
        result[i] = data[i] + other[i];
    }
    return result;
}

State State::operator-(const State& other) const {
    if (_size != other.data.size()) {
        throw std::runtime_error("Vector sizes do not match for substraction.");
    }
    State result(_size);
    for (size_t i = 0; i < _size; ++i) {
        result[i] = data[i] - other[i];
    }
    return result;
}


// Define the << operator outside the class
std::ostream& operator<<(std::ostream& os, const State& state) {
    os << "[";
    for (size_t i = 0; i < state._size; ++i) {
        os << state.data[i];
        if (i < state._size - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}


size_t State::size() const
{
    return _size;
}


std::vector<double> State::EigenToVector(const Eigen::VectorXd& eigen_vector) {
    std::vector<double> result(eigen_vector.data(), eigen_vector.data() + eigen_vector.size());
    return result;
}

//////// State4D_ECI ////////

// Constructor initializing with a 4D vector
State4D_ECI::State4D_ECI(std::vector<double>& initial_state) : State(initial_state) {
    if (initial_state.size() != 4) {
        throw std::runtime_error("State4D_ECI must have a 4D initial state.");
    }
}

// Constructor with initializer list
State4D_ECI::State4D_ECI(std::initializer_list<double> initial_state) : State(initial_state) 
{
    if (initial_state.size() != 4) {
        throw std::runtime_error("State4D_ECI must have a 4D initial state.");
    }
}

// Constructor initializing with zeros
State4D_ECI::State4D_ECI() : State(4) {}


//////// State6D ////////

// Constructor initializing with a 6D vector
State6D::State6D(std::vector<double>& initial_state) : State(initial_state) {
    if (initial_state.size() != 6) {
        throw std::runtime_error("State6D must have a 6D initial state.");
    }
}

// Constructor with initializer list
State6D::State6D(std::initializer_list<double> initial_state) : State(initial_state) {
if (initial_state.size() != 6) {
    throw std::runtime_error("State6D must have a 6D initial state.");
    }
}

// Constructor initializing with zeros
State6D::State6D() : State(6) {}

//////// State6D_ECI ////////

// Constructor initializing with a 4D vector
State6D_ECI::State6D_ECI(std::vector<double>& initial_state) : State(initial_state) {
    if (initial_state.size() != 6) {
        throw std::runtime_error("State6D_ECI must have a 6D initial state.");
    }
}

State6D_ECI::State6D_ECI(std::initializer_list<double> initial_state) : State(initial_state) {
    if (initial_state.size() != 6) {
        throw std::runtime_error("State6D_ECI must have a 6D initial state.");
    }
}

// Constructor initializing with zeros
State6D_ECI::State6D_ECI() : State(6) {}

//////// State6D_OE ////////

State6D_OE::State6D_OE(std::vector<double>& initial_state) : State(initial_state) {
    if (initial_state.size() != 6) {
        throw std::runtime_error("State6D_OE must have a 6D initial state.");
    }

    CheckOE(initial_state);

}


void State6D_OE::CheckOE(std::vector<double>& orbital_elements)
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


//////// Control ////////


Control::Control(std::vector<double>& initial_state) : State(initial_state) {}

Control::Control(size_t size) : State(size) {}

Control::Control(std::initializer_list<double> initial_state) : State(initial_state) {}

// Constructor initializing with an Eigen::VectorXd
Control::Control(const Eigen::VectorXd& initial_state) : State(initial_state) {}


//////// Control2D ////////

// Constructor initializing with a 2D vector
Control2D::Control2D(std::vector<double>& initial_state) : Control(initial_state) {
    if (initial_state.size() != 2) {
        throw std::runtime_error("Control2D must have a 2D initial state.");
    }
}

// Constructor with initializer list
Control2D::Control2D(std::initializer_list<double> initial_state) : Control(initial_state) {
    if (initial_state.size() != 2) {
        throw std::runtime_error("Control2D must have a 2D initial state.");
    }
}

// Constructor initializing with zeros
Control2D::Control2D() : Control(2) {}


//////// Control3D ////////

// Constructor initializing with a 3D vector
Control3D::Control3D(std::vector<double>& initial_state) : Control(initial_state) {
    if (initial_state.size() != 3) {
        throw std::runtime_error("Control3D must have a 3D initial state.");
    }
}

// Constructor with initializer list
Control3D::Control3D(std::initializer_list<double> initial_state) : Control(initial_state) {
    if (initial_state.size() != 3) {
        throw std::runtime_error("Control3D must have a 3D initial state.");
    }
}

// Constructor initializing with zeros
Control3D::Control3D() : Control(3) {}