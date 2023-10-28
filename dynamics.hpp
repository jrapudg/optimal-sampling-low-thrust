#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <vector>


#define MU 3.986004418e14
#define G 6.67430e-11
#define M 5.9722e24


class State 
{

public:

    // Constructors 
    State(std::vector<double>& initial_state) : data(initial_state) {};
    // Constructor initializing the vector with a size
    State(size_t size) : data(size) {};

    // Overload [] operator for element access
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    // Overload * operator for scalar multiplication
    State operator*(double scalar);

    // Overload + operator for element-wise addition
    State operator+(const State& other) const;

    // Overload << to output the state
    friend std::ostream& operator<<(std::ostream& os, const State& state);


private:
    std::vector<double> data;

};





class Spacecraft
{
    public:

        Spacecraft(double mass) : current_state(6), mass(mass) {};
        Spacecraft(double mass, const State& initial_state) : current_state(initial_state), mass(mass) {};
        Spacecraft(double mass, std::vector<double>& initial_state) : current_state(initial_state), mass(mass) {};

        State Dynamics(const State& x, const std::vector<double>& u);
        void RK4(const State& state, const std::vector<double>& u, double dt, State& next_state);

        void Advance(const std::vector<double>& u, double dt);

        const State& GetState() const;




    private:

        State current_state;
        double mass;


};






#endif