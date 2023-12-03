#ifndef PENDELUM_HPP
#define PENDELUM_HPP

#include <string>
#include <random>
#include <vector>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>


namespace Pendelum
{

typedef Eigen::VectorXd Vector;


//////// State ////////

typedef Eigen::Matrix<double, 2, 1> State;
typedef Eigen::Matrix<double, 1, 1> Control;

typedef Eigen::Matrix<double, 2, 2> MatrixA;
typedef Eigen::Matrix<double, 2, 1> MatrixB;


// Define pendelum constants
#define g_ 9.81 //gravity
#define l 1 //length of the pendelum
#define m 1 //mass of the pendelum

Vector toEigen(std::vector<double> v);
void pendelum_dynamics(const State& x, const Control& u, State& xdot); 
void sample_pendelum(State &sampled_state); 


void Print(const State& s, std::string name = "State");
void Print(const Control& c, std::string name = "Control");



}

#endif
