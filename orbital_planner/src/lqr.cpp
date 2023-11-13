
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <random>

#include "lqr.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace std::chrono;



LQR::LQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
 : Q(Q), R(R)
{

    if (Q.rows() != Q.cols() || R.rows() != R.cols()) {
        throw std::invalid_argument("Matrices Q and R must be square.");
    }

    dim = Q.rows();

    // TODO check Q is p.s.d, R is p.d

}


const Eigen::MatrixXd& LQR::getQ() const {
    return Q;
}

const Eigen::MatrixXd& LQR::getR() const {
    return R;
}

Eigen::MatrixXd LQR::ComputeCostMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B)
{
    // TODO ensure Q is symmetric and pd, R is pd
    // TODO Check dimensions on A and B 
    return _SolveRicatti(A, B);

}



Eigen::MatrixXd LQR::_SolveRicatti(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double tol)
{


    // E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
    // International Journal of Control, 77:8, 767-788, 2004. DOI: 10.1080/00207170410001714988
    // https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c 


    // Initialization 
    int j = 0;
    Eigen::MatrixXd Ak = A;
    Eigen::LLT<Eigen::MatrixXd> R_LLT(R); // compute the Cholesky decomposition of R
    Eigen::MatrixXd Gk = B * R_LLT.solve(B.transpose());
    Eigen::MatrixXd Hk = Q;
    Eigen::MatrixXd Hk1 = Q;

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
        Gk += Ak * V2 * Ak.transpose();

        // Hk1 = Hk + V1' Hk Ak
        Hk1 = Hk + V1.transpose() * Hk * Ak;

    } while ((Hk1-Hk).norm() > tol * Hk1.norm());


    return Hk1; // This is S


}



// Conversion to Eigen::VectorXd
Eigen::VectorXd LQR::toEigen(State state)
{
    int size =state.GetSize();
    Eigen::VectorXd vec(size);
    for (size_t i = 0; i < size; ++i) {
        vec[i] = state[i];
    }
    return vec;
}



double LQR::ComputeCostToGo(State state, State target_state, Eigen::MatrixXd& S)
{
    Eigen::MatrixXd xbar = toEigen(target_state - state);
    return  (xbar.transpose() * S * xbar)(0,0);
}

Control LQR::ComputeOptimalPolicy(State current_state, Eigen::MatrixXd& B, Eigen::MatrixXd& S)
{
    Eigen::LLT<Eigen::MatrixXd> R_LLT(R); // compute the Cholesky decomposition of R
    Eigen::VectorXd u = R_LLT.solve(B.transpose() * S * toEigen(current_state));
    return Control(u);
}

void Discretize(Eigen::MatrixXd& Ac, Eigen::MatrixXd& Bc, double dt, Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd)
{

    int nx = Ac.rows(); 
    int nu = Bc.cols(); 

    // Create a combined matrix for A and B
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + nu, nx + nu);
    M.topLeftCorner(nx, nx) = Ac * dt;
    M.topRightCorner(nx, nu) = Bc * dt;

    // Take the matrix exponential of M
    M = M.exp();

    // Extract Ad and Bd from the matrix exponential of M
    Ad = M.topLeftCorner(nx, nx);
    Bd = M.topRightCorner(nx, nu);

}


void clohessy_wiltshire(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double m=100.0) 
{

    double sma = LEO_MAX;
    const double n = std::sqrt(3.986004418e14 / std::pow(sma, 3));

    // Define matrix A
    A << 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1,
         3*n*n, 0, 0, 0, 2*n, 0,
         0, 0, 0, -2*n, 0, 0,
         0, 0, -n*n, 0, 0, 0;

    // Define matrix B
    B << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        1, 0, 0,
        0, 1, 0, 
        0, 0, 1;
    
    B /= m;

}



/*int main()
{
    
    double dt = 0.1;

    // Clohessy-Wiltshire 6D 
    
    Eigen::MatrixXd A(6,6);
    Eigen::MatrixXd B(6,3);
    Eigen::MatrixXd Ad(6,6);
    Eigen::MatrixXd Bd(6,3);
    clohessy_wiltshire(A, B);
    std::cout << "A: \n" << A << std::endl;
    std::cout << "B: \n" << B << std::endl;
    Discretize(A, B, dt, Ad, Bd);
    std::cout << "Ad: \n" << Ad << std::endl;
    std::cout << "Bd: \n" << Bd << std::endl;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6) * 10;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);


    // Create an LQR instance and compute the cost matrix
    LQR lqr(Q, R);
    auto start = high_resolution_clock::now();
    Eigen::MatrixXd S = lqr.ComputeCostMatrix(Ad, Bd); 
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Computed Cost Matrix (S): \n" << S << std::endl;
    std::cout << "Time taken by ricatti: " << duration.count() << " µs" << std::endl;


    std::random_device rd;
    std::mt19937 rng = std::mt19937(rd());
    std::vector<double> d_start(6);
    std::uniform_real_distribution<double> dis = std::uniform_real_distribution<double>(-100, 100);
    std::cout << "Start ";
    for (int i = 0; i < 6; ++i)
    {
        d_start[i] = dis(rng);
        std::cout << d_start[i] << " ";
    }
    std::cout << std::endl;


    //State6D s_state({-1, -4, -5, 0.1, 0.2, 0.3});
    State6D s_state(d_start);
    State6D g_state;


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

}*/