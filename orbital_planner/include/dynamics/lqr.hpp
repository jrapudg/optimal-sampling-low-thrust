#ifndef LQR_HPP
#define LQR_HPP 

#include <eigen3/Eigen/Dense>

#include "states.hpp"

class LQR 
{

    public:


        // Constructor
        LQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

        // const reference to avoid external modification 
        const Eigen::MatrixXd& getQ() const;
        const Eigen::MatrixXd& getR() const;



        Eigen::MatrixXd ComputeCostMatrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B);
        double ComputeCostToGo(State state, State target_state, Eigen::MatrixXd& S);
        Control ComputeOptimalPolicy(State current_state, Eigen::MatrixXd& B, Eigen::MatrixXd& S);


        Eigen::VectorXd toEigen(State state);



    private:

        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;

        int dim;


        Eigen::MatrixXd _SolveRicatti(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double tol=1e-10);
        



};






#endif 