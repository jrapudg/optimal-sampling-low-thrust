#include "states.hpp"


double& State::operator[](size_t index) {
    return data[index];
}

const double& State::operator[](size_t index) const {
    return data[index];
}

State State::operator*(double scalar) {
        State result(size);
        for (size_t i = 0; i < size; ++i) {
            result[i] = data[i] * scalar;
        }
        return result;
    }



State State::operator+(const State& other) const {
    if (size != other.data.size()) {
        throw std::runtime_error("Vector sizes do not match for addition.");
    }
    State result(size);
    for (size_t i = 0; i < size; ++i) {
        result[i] = data[i] + other[i];
    }
    return result;
}


// Define the << operator outside the class
std::ostream& operator<<(std::ostream& os, const State& state) {
    os << "[";
    for (size_t i = 0; i < state.size; ++i) {
        os << state.data[i];
        if (i < state.size - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}


size_t State::GetSize() const
{
    return size;
}
