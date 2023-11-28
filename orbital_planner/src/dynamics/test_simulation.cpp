#include "catch.hpp"
#include "simulation.hpp"



using namespace Simulation;



/*TEST_CASE("Test: Discretization of A and B", "[dicretization]") 
{


    const double n = std::sqrt(MU / std::pow(2000000, 3));
    MatrixA A;
    MatrixB B;
    A << 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1,
         0.000149475, 0, 0, 0, -0.0141174, 0,
         0, 0, 0, -0.0141174, 0, 0,
         0, 0, -4.98251e-5, 0, 0, 0;
    
    B << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0.01, 0, 0,
        0, 0.01, 0, 
        0, 0, 0.01;



    double dt = 0.2;
    MatrixA A_try;
    MatrixB B_try;
    Simulator::Discretize(A, B, dt, A_try, B_try);

    
    MatrixA Ad;
    MatrixB Bd;

    Ad <<  1.0,        0.0,        0.0,          0.2,          0.000282347, 0.0,
            -2.8136e-9, 1.0,        0.0,         -0.000282347, 0.2,          0.0,
            0.0,        0.0,        0.999999,     0.0,          0.0,          0.2,
            2.9895e-5,  0.0,        0.0,          0.999999,     0.00282347,   0.0,
            -4.22039e-8,0.0,        0.0,         -0.00282347,   0.999996,     0.0,
            0.0,        0.0,       -9.96501e-6,   0.0,          0.0,          0.999999;
    
    Bd <<   0.0002,      1.88232e-7,  0.0,
            -1.88232e-7,  0.0002,      0.0,
            0.0,          0.0,         0.0002,
            0.002,        2.82347e-6,  0.0,
            -2.82347e-6,  0.002,       0.0,
            0.0,          0.0,         0.002;

    using namespace std;
    cout << Ad << endl;
    cout << "----" << endl;
    cout << A_try << endl;
    cout << "----" << endl;
    cout << Bd << endl;
    cout << "----" << endl;
    cout << B_try << endl;

    double eps = 1e-4;    
    //CHECK(A_try.isApprox(Ad, 1e-3));  // Uses Eigen's isApprox with a specified tolerance
    //CHECK(B_try.isApprox(Bd, 1e-3));
    for (int i = 0; i < A_try.rows(); ++i) 
    {
        for (int j = 0; j < A_try.cols(); ++j) {
            CHECK(A_try(i, j) == Approx(Ad(i, j)).epsilon(eps)); 
        }
    }

    for (int i = 0; i < B_try.rows(); ++i) {
        for (int j = 0; j < B_try.cols(); ++j) {
            CHECK(B_try(i, j) == Approx(Bd(i, j)).epsilon(eps));  
        }
    }


}*/