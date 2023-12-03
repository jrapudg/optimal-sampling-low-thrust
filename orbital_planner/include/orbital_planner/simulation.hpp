#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <functional>
#include <eigen3/Eigen/Dense>
#include "orbital_planner/astrodynamics.hpp"



namespace Simulation
{

using namespace Astrodynamics;



class Simulator
{

public:
    // Constructor
    Simulator(std::function<void(const State&, const Control&, State&)> dynamics, double dt);


    void Step(const State& state, const Control& control, State& next_state);

    static void Discretize(const MatrixA& Ac, const MatrixB& Bc, double dt, MatrixA& Ad, MatrixB& Bd);

private:

    double dt;
    std::function<void(const State&, const Control&, State&)> Dynamics;

    void RK4(const State& state, const Control& control, State &next_state);


};


}
#endif 