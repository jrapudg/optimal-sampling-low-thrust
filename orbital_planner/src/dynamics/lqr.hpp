// #ifndef LQR_HPP
// #define LQR_HPP 

// #include <eigen3/Eigen/Dense>
// //#include "astrodynamics.hpp"
// #include "pendelum.hpp"
// #include "simulation.hpp"

// namespace Optimal
// {

// //using namespace Astrodynamics;

// using namespace Pendelum; 

// typedef Eigen::Matrix<double, 2, 2> MatrixS;
// typedef Eigen::Matrix<double, 2, 2> MatrixQ;
// typedef Eigen::Matrix<double, 1, 1> MatrixR;
// typedef Eigen::Matrix<double, 1, 2> MatrixK;


// class LQR 
// {

//     public:
//         // Constructors
//         void ComputeCostMatrix(const MatrixA& A, const MatrixB& B, Optimal::MatrixS& S, double tol=1e-12, bool DEBUG=false);

//         static double ComputeCostToGo(State state, State target_state, const Optimal::MatrixS& S);

//         Optimal::Control ComputeOptimalPolicy(State current_state, MatrixA& A, MatrixB& B, Optimal::MatrixS& S);

//         Optimal::MatrixK ComputeOptimalGain(MatrixA& A, MatrixB& B, Optimal::MatrixS& S);

//         double QuadraticCost(State& state, Control& control);

//         double GetTrajectoryCost(State starting_state, State& goal_state, MatrixA& A, MatrixB& B, Simulation::Simulator& sim, double tol=1e-2);
        

// };

// }

// #endif 

#ifndef LQR_HPP
#define LQR_HPP 

#include <eigen3/Eigen/Dense>
#include "pendelum.hpp"
#include "simulation.hpp"

namespace Optimal
{

using namespace Pendelum;
typedef Eigen::Matrix<double, 2, 2> MatrixS;
typedef Eigen::Matrix<double, 2, 2> MatrixQ;
typedef Eigen::Matrix<double, 1, 1> MatrixR;
typedef Eigen::Matrix<double, 1, 2> MatrixK;


class LQR 
{

    public:
        // Constructors
        LQR(const MatrixQ& Q, const MatrixR& R);

        // Getters
        const MatrixQ& GetQ() const;
        const MatrixR& GetR() const;

        // Setters
        void SetQ(const MatrixQ& newQ);
        void SetR(const MatrixR& newR);

        void ComputeCostMatrix(const MatrixA& A, const MatrixB& B, MatrixS& S, double tol=1e-12, bool DEBUG=false);

        static double ComputeCostToGo(State state, State target_state, const MatrixS& S);

        Control ComputeOptimalPolicy(State current_state, MatrixA& A, MatrixB& B, MatrixS& S);

        MatrixK ComputeOptimalGain(MatrixA& A, MatrixB& B, MatrixS& S);

        double QuadraticCost(State& state, Control& control);

        double GetTrajectoryCost(State starting_state, State& goal_state, MatrixA& A, MatrixB& B, Simulation::Simulator& sim, double tol=1e-2);


    private:

        MatrixQ Q;
        MatrixR R;

        int dim;

        void _SolveRicatti(const MatrixA& A, const MatrixB& B, MatrixS& S, double tol, bool DEBUG=false);
        

};






}






#endif 