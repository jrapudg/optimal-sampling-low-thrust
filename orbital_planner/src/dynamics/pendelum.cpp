#include "pendelum.hpp"

namespace Pendelum
{

Vector toEigen(std::vector<double> v)
{
    int size = v.size();
    Vector vec(size);
    for (size_t i = 0; i < size; ++i) {
        vec[i] = v[i];
    }
    return vec;
}

void pendelum_dynamics(const State& x, const Control& u, State& xdot)
{
    //Velocity 
    xdot[0] = x[1];

    //Acceleration 
    xdot[1] = (-g_/l)*sin(x[0]) + (1/(m*l*l))*u[0];

}

void sample_pendelum(State &sampled_state)
{
    
    //create random device
    std::random_device rd; 
    std::mt19937 gen(rd());

    //create a uniform distribution for the angle
    std::uniform_real_distribution<double> angle_distribution(-M_PI, M_PI);

    //create a uniform distribution for the angular velocity
    std::uniform_real_distribution<double> angular_vel_distribution(-2*M_PI, 2*M_PI);

    sampled_state[0] = angle_distribution(gen);

    sampled_state[1] = angular_vel_distribution(gen); 
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

}