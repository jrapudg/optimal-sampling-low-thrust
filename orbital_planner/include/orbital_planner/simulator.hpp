#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <functional>


#include "states.hpp"

// TODO Eigen for matrix math?

class Simulator
{

public:
    // Constructor
    Simulator(std::function<void(State, Control, State&)> dynamics, double dt);



    void Step(State state, Control control, State &next_state);

    //void Linearize(State state, Control control, Matrix& A, Matrix& B);



private:

    double dt;
    std::function<void(State, Control, State&)> Dynamics;


    void RK4(State state, Control control, State &next_state);




};



#endif 