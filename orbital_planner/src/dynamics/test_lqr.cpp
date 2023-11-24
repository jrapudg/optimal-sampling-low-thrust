#include "catch.hpp"
#include <chrono>

#include "lqr.hpp"


using namespace std::chrono;
using namespace Optimal;



TEST_CASE( "Test: LQR class constructor", "[lqr_constructor]" ) 
{

    // Ensure that if Q and R are not p.s.d and p.d respectively, it throws an error 

    MatrixQ Q_valid;
    Q_valid << 8.76523, 1.52344, 1.86413, 2.12904, 2.26651, 2.00696,
            1.52344, 7.36029, 1.17142, 1.46854, 1.66219, 1.36636,
            1.86413, 1.17142, 7.71248, 1.85719, 1.7382,  1.51935,
            2.12904, 1.46854, 1.85719, 8.17972, 2.07346, 1.83349,
            2.26651, 1.66219, 1.7382,  2.07346, 8.30063, 1.83647,
            2.00696, 1.36636, 1.51935, 1.83349, 1.83647, 8.15814;

    MatrixQ Q_invalid;
    Q_invalid <<  7.55518,    0.87338,   0.393571,  1.46721,   0.75049,    1.10524,
                0.87338,    5.2454 ,   1.38436,   0.23903,   -0.826661,   1.17527,
                0.393571,   1.38436,   4.38098,   1.00554,    1.47492,    0.364326,
                1.46721,    0.23903,   1.00554,   7.63733,    0.687635,   0.785417,
                0.75049,   -0.826661,  1.47492,   0.687635,   4.27385,    2.06897,
                1.10524,    1.17527,   0.364326,  0.785417,   2.06897,   -0.005;
    

    MatrixR R_valid;
    R_valid << 3.62149,   0.571179,  0.533249,
            0.571179,  4.02484,  0.487787,
            0.533249,  0.487787,  4.00794;

    MatrixR R_invalid;
    R_invalid << 2.60554,   1.58,      -0.611035,
                0.378746,  3.40225,    0.784753,
                0.37428,   0.813144,   2.43279;

    MatrixR R_invalid_neg;
    R_invalid_neg << -1.37851,    0.571179,   0.533249,
                    0.571179,  -0.975164,   0.487787,
                    0.533249,   0.487787,  -0.99206;


    REQUIRE_NOTHROW(LQR(Q_valid, R_valid));
    CHECK_THROWS_AS( LQR(Q_valid, R_invalid), std::runtime_error );
    CHECK_THROWS_AS( LQR(Q_valid, R_invalid_neg), std::runtime_error );
    CHECK_THROWS_AS( LQR(Q_invalid, R_valid), std::runtime_error );

    // TODO check also the getter and setter

}




TEST_CASE( "Test: Ricatti solve and subsequent computation", "[ricatti]" ) 
{
        
        // Set-up
        MatrixQ Q = Eigen::MatrixXd::Identity(6, 6) * 5;
        MatrixR R = Eigen::MatrixXd::Identity(3, 3);
        LQR lqr(Q,R);

        // Discretized matrix - dt=0.2s
        MatrixA A;
        MatrixB B;
        A <<  1.0,        0.0,        0.0,          0.2,          0.000282347, 0.0,
        -2.8136e-9, 1.0,        0.0,         -0.000282347, 0.2,          0.0,
        0.0,        0.0,        0.999999,     0.0,          0.0,          0.2,
        2.9895e-5,  0.0,        0.0,          0.999999,     0.00282347,   0.0,
        -4.22039e-8,0.0,        0.0,         -0.00282347,   0.999996,     0.0,
        0.0,        0.0,       -9.96501e-6,   0.0,          0.0,          0.999999;
        B <<   0.0002,      1.88232e-7,  0.0,
        -1.88232e-7,  0.0002,      0.0,
        0.0,          0.0,         0.0002,
        0.002,        2.82347e-6,  0.0,
        -2.82347e-6,  0.002,       0.0,
        0.0,          0.0,         0.002;

        MatrixS S_true;
        // from MATLAB dare
        S_true << 0.024132270527579,  -0.000005275627377,  -0.000000000000000,   0.112306669743035,   0.007426811370491, -0.000000000000000,
        -0.000005275627377,   0.024053097537894,  -0.000000000000000,  -0.007426645439797,   0.111556745493081, -0.000000000000001,
        -0.000000000000000,  -0.000000000000000,   0.024000563502725,  -0.000000000000000,  -0.000000000000000, 0.111554812136006,
        0.112306669743035,  -0.007426645439797,  -0.000000000000000,   1.065933561824438,   0.000116796689339, 0.000000000000000,
        0.007426811370491,   0.111556745493081,  -0.000000000000000,   0.000116796689339,   1.062424458004297, -0.000000000000003,
        -0.000000000000000,  -0.000000000000001,   0.111554812136006,   0.000000000000000,  -0.000000000000003, 1.062407508470502;
        S_true *= 1e4;


        SECTION("Ricatti recursion") 
        {
                MatrixS S;
                auto start = high_resolution_clock::now();

                lqr.ComputeCostMatrix(A, B, S);

                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                std::cout << "Time taken by Ricatti recursion: " << duration.count() << " µs" << std::endl;
                std::cout << "S_true" << std::endl;
                std::cout << S_true << std::endl;
                std::cout << "S" << std::endl;
                std::cout << S << std::endl;

                CHECK(S.isApprox(S_true, 1e-4));
        }


        MatrixK K_true;

        K_true << 2.199077819592401,  -0.145416866728344,  -0.000000000000001,  21.084068133289264,   0.017255490065267,  -0.000000000000007,
                0.145420114936813,   2.184235461125166,  -0.000000000000002,  -0.012681632677425,  21.015358280303726,  -0.000000000000050,
                -0.000000000000001,  -0.000000000000009,   2.184094382424176,   0.000000000000007,  -0.000000000000099,  21.015033527580208;

        State current_state = {18.5, 13, -7.2, -1.8, -0.5, -1.3};
        Control optimal_policy = {-0.832570010037644, -20.600480919625788, 43.045023139308441};
 
  
        SECTION("Optimal policy")
        {
                
                MatrixK K = lqr.ComputeOptimalGain(A, B, S_true);

                CHECK(K.isApprox(K_true, 1e-4));
                std::cout << "K_true" << std::endl;
                std::cout << K_true << std::endl;
                std::cout << "K" << std::endl;
                std::cout << K << std::endl;

                Control u = lqr.ComputeOptimalPolicy(current_state, A, B, S_true);
                CHECK(u.isApprox(optimal_policy, 1e-4));

        }



        






}



