
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <random>
#include <unistd.h>


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

double LQR::ComputeCostToGo(State state, State target_state, const MatrixS& S)
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

double LQR::QuadraticCost(State& state, Control& control)
{
    return (state.transpose() * Q * state + control.transpose() * R * control)(0);
}

double LQR::GetTrajectoryCost(State starting_state, State& goal_state, MatrixA& A, MatrixB& B, Simulation::Simulator& sim, double tol)
{   
    std::cout << "Getting Trajectory cost ..." << std::endl;
    std::cout << "Start ";
    Print(starting_state);
    std::cout << "Goal ";
    Print(goal_state); 
    Control control;

    // Compute the Cost Matrix 
    MatrixS S;
    ComputeCostMatrix(A, B, S);
    // Compute Optimal Gain
    MatrixK K = ComputeOptimalGain(A, B, S);

    // Initial state
    State current_state = starting_state;
    State next_state;

    // Initial cost     
    double J = 0;
    control = - K * (current_state - goal_state);
    J += QuadraticCost(current_state, control);


    while (true)
    {
        //std::cout << "-----------" << std::endl;
        //std::cout << "Current state: \n" << current_state << std::endl;
        // Optimal policy
        control = - K * (current_state - goal_state);

        //std::cout << "Control: \n" << control << std::endl;

        sim.Step(current_state, control, next_state);
        //next_state = A * current_state + B * control;

        Print(next_state);

        J += QuadraticCost(next_state, control);

        std::cout << "Residual " << (current_state - goal_state).squaredNorm() << std::endl;

        if ((current_state - goal_state).squaredNorm() <= tol)
        {
            break;
        }
        else
        {
            current_state = next_state;
        }

        //std::cout << "J --- " << J << std::endl;
        usleep(1000);


    }

    return J;

}

}

/*
int main()
{

    using namespace std;
    using namespace Astrodynamics;
    using namespace Optimal;
    using namespace Simulation;

    double dt = 1;

    // Clohessy-Wiltshire 6D 
    
    MatrixA A(6,6);
    MatrixB B(6,3);
    MatrixA Ad(6,6);
    MatrixB Bd(6,3);
    GetClohessyWiltshireMatrices(A, B);

    //std::cout << "A: \n" << A << std::endl;
    //std::cout << "B: \n" << B << std::endl;
    Simulator::Discretize(A, B, dt, Ad, Bd);
    //std::cout << "Ad: \n" << Ad << std::endl;
    //std::cout << "Bd: \n" << Bd << std::endl;

    MatrixQ Q = Eigen::MatrixXd::Identity(6, 6);
    MatrixR R = Eigen::MatrixXd::Identity(3, 3);


    // Create an LQR instance and compute the cost matrix
    LQR lqr(Q, R);
    //auto start = high_resolution_clock::now();
    //auto stop = high_resolution_clock::now();
    //auto duration = duration_cast<microseconds>(stop - start);


    Simulator sim(ClohessyWiltshire, dt);
    //State starting_state = {5.0, 2.0, -1.0, -0.2, 0.0, -0.49};

    State starting_state = {-2.17495, 2.00354, 0.211689, 0.20699, -0.134153, -0.0807519};
    //State goal_state = {0.0,0.0,0.0,0,0,0};
    State goal_state = {-3, 2, 1.3, -0.2, 0.3, -0.2};

    double J = lqr.GetTrajectoryCost(starting_state, goal_state, Ad, Bd, sim);
    std::cout << J << std::endl;

    return 0;

}


*/

