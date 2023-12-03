#include <iostream>
#include <cmath>
#include <vector>

#include "jacobians_pendelum.hpp"

using namespace Pendelum;

void GetAc_pendelum(MatrixA& Ac, const State& x, const Control& u)
{
    Ac(0,0) = 0; 
    Ac(0,1) = 1; 
    Ac(1, 0) = -(g_/l)*cos(x[0]); 
    Ac(1,1) = 0; 
}

void GetBc_pendelum(MatrixB& Bc, const State& x, const Control& u)
{
    Bc(0,0) = 0; 
    Bc(1, 0)= 1/(m*l*l); 
}