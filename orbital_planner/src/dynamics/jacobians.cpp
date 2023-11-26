#include <iostream>
#include <cmath>
#include <vector>

#include "jacobians.hpp"

using namespace Astrodynamics;


//CR3BP continuous dynamics jacobians for the Earth Moon system
void GetJacobiansCR3BP(MatrixA& Ac, MatrixB& Bc, const State& x, const Control& u)
{
    GetAc_CR3BP(Ac, x, u);
    GetBc_CR3BP(Bc, x, u);
}



void GetAc_CR3BP(MatrixA& Ac, const State& x, const Control& u)
{

    Ac(0,0) = 0;
    Ac(1,0) = 0;
    Ac(2,0) = 0;
    Ac(3,0) = 1 + -0.98785 / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -0.01215 / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -1.5 * ((-0.0120023775 + -0.98785 * x(0)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * (0.0243 + 2 * x(0)) + -1.5 * ((0.0120023775 + -0.01215 * x(0)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * (-1.9757 + 2 * x(0));
    Ac(4,0) = -1.5 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.01215 * x(1)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * (-1.9757 + 2 * x(0)) + -1.5 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(1)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * (0.0243 + 2 * x(0));
    Ac(5,0) = -1.5 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * (-1.9757 + 2 * x(0)) * ((-0.01215 * x(2)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) + -1.5 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(2)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * (0.0243 + 2 * x(0));
    Ac(0,1) = 0;
    Ac(1,1) = 0;
    Ac(2,1) = 0;
    Ac(3,1) = -3.0 * ((-0.0120023775 + -0.98785 * x(0)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * x(1) + -3.0 * ((0.0120023775 + -0.01215 * x(0)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * x(1);
    Ac(4,1) = 1 + -0.98785 / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -0.01215 / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -3.0 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(1)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(1) + -3.0 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.01215 * x(1)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(1);
    Ac(5,1) = -3.0 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.01215 * x(2)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(1) + -3.0 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(2)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(1);
    Ac(0,2) = 0;
    Ac(1,2) = 0;
    Ac(2,2) = 0;
    Ac(3,2) = -3.0 * ((-0.0120023775 + -0.98785 * x(0)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * x(2) + -3.0 * ((0.0120023775 + -0.01215 * x(0)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * x(2);
    Ac(4,2) = -3.0 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(1)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(2) + -3.0 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.01215 * x(1)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(2);
    Ac(5,2) = -0.98785 / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -0.01215 / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 1.5) + -3.0 * pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.98785 * x(2)) / pow(pow(0.01215 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(2) + -3.0 * pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 0.5) * ((-0.01215 * x(2)) / pow(pow(-0.98785 + x(0), 2) + pow(x(1), 2) + pow(x(2), 2), 3.0)) * x(2);
    Ac(0,3) = 1;
    Ac(1,3) = 0;
    Ac(2,3) = 0;
    Ac(3,3) = 0;
    Ac(4,3) = -2;
    Ac(5,3) = 0;
    Ac(0,4) = 0;
    Ac(1,4) = 1;
    Ac(2,4) = 0;
    Ac(3,4) = 2;
    Ac(4,4) = 0;
    Ac(5,4) = 0;
    Ac(0,5) = 0;
    Ac(1,5) = 0;
    Ac(2,5) = 1;
    Ac(3,5) = 0;
    Ac(4,5) = 0;
    Ac(5,5) = 0;
}

void GetBc_CR3BP(MatrixB& Bc, const State& x, const Control& u)
{
    Bc(0,0) = 0;
    Bc(1,0) = 0;
    Bc(2,0) = 0;
    Bc(3,0) = 1;
    Bc(4,0) = 0;
    Bc(5,0) = 0;
    Bc(0,1) = 0;
    Bc(1,1) = 0;
    Bc(2,1) = 0;
    Bc(3,1) = 0;
    Bc(4,1) = 1;
    Bc(5,1) = 0;
    Bc(0,2) = 0;
    Bc(1,2) = 0;
    Bc(2,2) = 0;
    Bc(3,2) = 0;
    Bc(4,2) = 0;
    Bc(5,2) = 1;
}


void GetJacobians2BP(MatrixA& Ac, MatrixB& Bc, const State& x, const Control& u)
{
    GetAc_2BP(Ac, x, u);
    GetBc_2BP(Bc, x, u);
}



void GetAc_2BP(MatrixA& Ac, const State& x, const Control& u)
{
    Ac(0,0) = 0;
    Ac(1,0) = 0;
    Ac(2,0) = 0;
    Ac(3,0) = -3.985e14 / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 3) + -3 * ((-3.985e14 * x(0)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(0);
    Ac(4,0) = -3 * ((-3.985e14 * x(1)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(0);
    Ac(5,0) = -3 * ((-3.985e14 * x(2)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(0);
    Ac(0,1) = 0;
    Ac(1,1) = 0;
    Ac(2,1) = 0;
    Ac(3,1) = -3 * ((-3.985e14 * x(0)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(1);
    Ac(4,1) = -3.985e14 / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 3) + -3 * ((-3.985e14 * x(1)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(1);
    Ac(5,1) = -3
     * ((-3.985e14 * x(2)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(1);
    Ac(0,2) = 0;
    Ac(1,2) = 0;
    Ac(2,2) = 0;
    Ac(3,2) = -3 * ((-3.985e14 * x(0)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(2);
    Ac(4,2) = -3 * ((-3.985e14 * x(1)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(2);
    Ac(5,2) = -3.985e14 / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 3) + -3 * ((-3.985e14 * x(2)) / pow(sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))), 6)) * sqrt(std::norm(x(0)) + std::norm(x(1)) + std::norm(x(2))) * x(2);
    Ac(0,3) = 1;
    Ac(1,3) = 0;
    Ac(2,3) = 0;
    Ac(3,3) = 0;
    Ac(4,3) = 0;
    Ac(5,3) = 0;
    Ac(0,4) = 0;
    Ac(1,4) = 1;
    Ac(2,4) = 0;
    Ac(3,4) = 0;
    Ac(4,4) = 0;
    Ac(5,4) = 0;
    Ac(0,5) = 0;
    Ac(1,5) = 0;
    Ac(2,5) = 1;
    Ac(3,5) = 0;
    Ac(4,5) = 0;
    Ac(5,5) = 0;

}

void GetBc_2BP(MatrixB& Bc, const State& x, const Control& u)
{
    Bc(0,0) = 0;
    Bc(1,0) = 0;
    Bc(2,0) = 0;
    Bc(3,0) = 1;
    Bc(4,0) = 0;
    Bc(5,0) = 0;
    Bc(0,1) = 0;
    Bc(1,1) = 0;
    Bc(2,1) = 0;
    Bc(3,1) = 0;
    Bc(4,1) = 1;
    Bc(5,1) = 0;
    Bc(0,2) = 0;
    Bc(1,2) = 0;
    Bc(2,2) = 0;
    Bc(3,2) = 0;
    Bc(4,2) = 0;
    Bc(5,2) = 1;
    
}


/*int main()
{
    // Test against ForwardDiff in Julia
    MatrixA Ac;
    MatrixB Bc;
    State x;
    Control u;

    //initialize the state (for cr3bp)
    //x << 1.1201297302380415,0,0.014654708958207016,0,0.17331212810099958,0;

    //initialize the state (for 2bp)
    x << -6.568741989057123e6,-1.9023114030561317e6,-697065.2121487064,-763.850262000727,-132.345846649287,7573.5676513547705;


    GetJacobians2BP(Ac, Bc, x, u);

    std::cout << Ac << std::endl;
    std::cout << Bc << std::endl;
    
    return 0;
}*/
