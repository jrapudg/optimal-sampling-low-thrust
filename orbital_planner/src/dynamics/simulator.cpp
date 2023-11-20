#include "simulator.hpp"



Simulator::Simulator(std::function<void(State, Control, State&)> dynamics, double dt) 
: Dynamics(dynamics), dt(dt)
{

}


void Simulator::Step(State state, Control control, State &next_state)
{

    RK4(state, control, next_state);
}


void Simulator::RK4(State state, Control control, State &next_state)
{
    size_t sz = state.size();
    State k1(sz), k2(sz), k3(sz), k4(sz);

    Dynamics(state, control, k1);
    Dynamics(state + k1 * 0.5, control, k2);
    Dynamics(state + k2 * 0.5, control, k3);
    Dynamics(state + k3, control, k4);

    next_state = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0); 

}