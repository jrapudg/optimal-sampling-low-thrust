
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <random>

#include "simulation.hpp"
#include "lqr.hpp"



namespace Optimal
{



using namespace std::chrono;


LQR::LQR(const MatrixQ& Q, const MatrixR& R)
 : Q(Q), R(R)
{

    if (Q.rows() != Q.cols() || R.rows() != R.cols()) {
        throw std::invalid_argument("Matrices Q and R must be square.");
    }

    dim = Q.rows(); // must be 6

    // TODO check Q is p.s.d, R is p.d

    /*
    if (!A.isApprox(A.transpose()) || A_llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Possibly non semi-positive definitie matrix!");
}    
    */

}


const MatrixQ& LQR::GetQ() const {
    return Q;
}

const MatrixR& LQR::GetR() const {
    return R;
}

void LQR::SetQ(const MatrixQ& newQ) {
    // Must check before
    Q = newQ;
}

void LQR::SetR(const MatrixR& newR) {
    // Must check before
    R = newR;
}





void LQR::ComputeCostMatrix(const MatrixA& A, const MatrixB& B, MatrixS& S,double tol, bool DEBUG)
{
    // TODO ensure Q is symmetric and pd, R is pd
    _SolveRicatti(A, B, S, tol, DEBUG);
}


void LQR::_SolveRicatti(const MatrixA& A, const MatrixB& B, MatrixS& S, double tol, bool DEBUG)
{


    // E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
    // International Journal of Control, 77:8, 767-788, 2004. DOI: 10.1080/00207170410001714988
    // https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c 

    // Initialization 
    /*int j = 0;
    MatrixA Ak = A;
    Eigen::LLT<MatrixR> R_LLT(R); // compute the Cholesky decomposition of R
    MatrixS Gk = B * R_LLT.solve(B.transpose());
    MatrixS Hk;
    MatrixS Hk1 = Q;

    // SDA algorithm
    do
    {
        // W = I + Gk Hk
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(Hk.rows(), Hk.cols()) + Gk * Hk;

        // LU decomposition for more efficient subsequent solve
        auto Wlu = W.lu();

        // Solve for V1 from from W V1 = Ak
        Eigen::MatrixXd V1 = Wlu.solve(Ak);

        // Solve for V2 from V2 W' = Gk
        Eigen::MatrixXd V2 = Wlu.solve(Gk.transpose()).transpose();

        // Ak1 = Ak V1
        Ak = Ak * V1;

        // Gk1 = Gk + Ak V2 Ak' 
        Gk = Gk + Ak * V2 * Ak.transpose();

        // Hk1 = Hk + V1' Hk Ak
        Hk1 = Hk + V1.transpose() * Hk * Ak;

    } while ((Hk1-Hk).norm() > tol * Hk1.norm());


    //return Hk1; // This is S
    S = Hk1;*/

    

    // Ricatti recursion
    MatrixA At = A.transpose();
    Eigen::Matrix<double, 3, 6> Bt = B.transpose();

    S = Q;
    MatrixS Sp;

    double d = tol;

    size_t i = 0;
    

    for (size_t ii=0; ii < 1000 && d >= tol; ++ii)
    {
        Sp = S;
        auto tlu = (Bt * S * B + R).lu();
        
        S = Q + At * S * A - At * S * B * tlu.solve(Bt * S * A);
        d = (S - Sp).array().abs().sum();

        if (DEBUG){i = ii;};
    }
    if (DEBUG)
    {
        std::cout << "Tolerance obtained: " << d << std::endl;
        std::cout << "Iteration count: " << i << std::endl;
    }



}


double LQR::ComputeCostToGo(State state, State target_state, MatrixS& S)
{
    State xbar = target_state - state;
    return  (xbar.transpose() * S * xbar)(0,0);
}


MatrixK LQR::ComputeOptimalGain(MatrixA& A, MatrixB& B, MatrixS& S)
{
    Eigen::Matrix<double, 3, 6> Bt = B.transpose();
    Eigen::LDLT<Eigen::MatrixXd> r_LLT(R + Bt * S * B); // compute the Cholesky decomposition
    return (r_LLT).solve(Bt * S * A);
}


Control LQR::ComputeOptimalPolicy(State current_state, MatrixA& A, MatrixB& B, MatrixS& S)
{
    return - ComputeOptimalGain(A, B, S) * current_state;
}







}


/*
int main()
{

    using namespace std;
    using namespace Astrodynamics;
    using namespace Optimal;
    using namespace Simulation;

    double dt = 0.1;

    // Clohessy-Wiltshire 6D 
    
    MatrixA A(6,6);
    MatrixB B(6,3);
    MatrixA Ad(6,6);
    MatrixB Bd(6,3);
    GetClohessyWiltshireMatrices(A, B);

    std::cout << "A: \n" << A << std::endl;
    std::cout << "B: \n" << B << std::endl;
    Simulator::Discretize(A, B, dt, Ad, Bd);
    std::cout << "Ad: \n" << Ad << std::endl;
    std::cout << "Bd: \n" << Bd << std::endl;

    MatrixQ Q = Eigen::MatrixXd::Identity(6, 6) * 5;
    MatrixR R = Eigen::MatrixXd::Identity(3, 3);


    // Create an LQR instance and compute the cost matrix
    LQR lqr(Q, R);
    auto start = high_resolution_clock::now();
    MatrixS S;
    lqr.ComputeCostMatrix(Ad, Bd, S); 
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Computed Cost Matrix (S): \n" << S << std::endl;
    std::cout << "Time taken by ricatti: " << duration.count() << " µs" << std::endl;


    std::random_device rd;
    std::mt19937 rng = std::mt19937(rd());
    State s_state;
    std::uniform_real_distribution<double> dis = std::uniform_real_distribution<double>(-100, 100);
    for (int i = 0; i < 6; ++i)
    {
        s_state[i] = dis(rng);
    }
    Print(s_state);

    State g_state;


    start = high_resolution_clock::now();
    double v = lqr.ComputeCostToGo(s_state, g_state, S);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "V(x) = " << v << std::endl;
    std::cout << "Time taken by CostToGo: " << duration.count() << " µs" << std::endl;
    


    start = high_resolution_clock::now();
    Control ustar = lqr.ComputeOptimalPolicy(s_state, Bd, S);
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    std::cout << "u* = " << ustar << std::endl;
    std::cout << "Time taken by ComputeOptimalPolicy: " << duration.count() << " µs" << std::endl;
    

    return 0;

}

*/