#ifndef LQR_HPP
#define LQR_HPP 

#include <eigen3/Eigen/Dense>
#include "astrodynamics.hpp"


namespace Optimal
{

using namespace Astrodynamics;
typedef Eigen::Matrix<double, 6, 6> MatrixS;
typedef Eigen::Matrix<double, 6, 6> MatrixQ;
typedef Eigen::Matrix<double, 3, 3> MatrixR;
typedef Eigen::Matrix<double, 3, 6> MatrixK;


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

        double ComputeCostToGo(State state, State target_state, MatrixS& S);

        Control ComputeOptimalPolicy(State current_state, MatrixA& A, MatrixB& B, MatrixS& S);

        MatrixK ComputeOptimalGain(MatrixA& A, MatrixB& B, MatrixS& S);

        double QuadraticCost(State& state, Control& control);



    private:

        MatrixQ Q;
        MatrixR R;

        int dim;

        void _SolveRicatti(const MatrixA& A, const MatrixB& B, MatrixS& S, double tol, bool DEBUG=false);
        

};






}






#endif 