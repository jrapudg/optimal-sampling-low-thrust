#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "simulation.hpp"


namespace Simulation
{

Simulator::Simulator(std::function<void(const State&, const Control&, State&)> dynamics, double dt) 
: Dynamics(dynamics), dt(dt)
{

}


void Simulator::Step(const State& state, const Control& control, State &next_state)
{
    RK4(state, control, next_state);
}


void Simulator::RK4(const State& state, const Control& control, State &next_state)
{
    State k1, k2, k3, k4;

    Dynamics(state, control, k1);
    Dynamics(state + k1 * 0.5, control, k2);
    Dynamics(state + k2 * 0.5, control, k3);
    Dynamics(state + k3, control, k4);

    next_state = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0); 

}




void Simulator::Discretize(const MatrixA& Ac, const MatrixB& Bc, double dt, MatrixA& Ad, MatrixB& Bd)
{

    int nx = Ac.rows(); 
    int nu = Bc.cols(); 

    // Create a combined matrix for A and B
    Eigen::MatrixXd Mat = Eigen::MatrixXd::Zero(nx + nu, nx + nu);
    Mat.topLeftCorner(nx, nx) = Ac * dt;
    Mat.topRightCorner(nx, nu) = Bc * dt;

    // Take the matrix exponential of M
    Mat = Mat.exp();

    // Extract Ad and Bd from the matrix exponential of M
    Ad = Mat.topLeftCorner(nx, nx);
    Bd = Mat.topRightCorner(nx, nu);

}


}

/*
int main()
{

    using namespace std;
    using namespace Astrodynamics;
    using namespace Simulation;
    
    State s1 = {1.0,0.2,1.0,1.0,1.0,2.0};
    State s2 = {1.0,1.0,1.0,1.0,1.0,1.0};

    CheckOrbitalElements(s1);

    Print(s1);
    Print(s2);

    Simulator sim(ClohessyWiltshire, 0.1);
    Control u = {50,50,35};
    Print(u);
    sim.Step(s1, u, s2);
    Print(s2);



}

*/