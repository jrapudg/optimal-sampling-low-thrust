#ifndef STATE_HPP
#define STATE_HPP 

#include <vector>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

// Constants for minimum and maximum altitudes (in kilometers) for LEO, MEO, and GEO
#define LEO_MIN 160.0;  // Minimum altitude for LEO
#define LEO_MAX 2000.0; // Maximum altitude for LEO

#define MEO_MIN 2000.0; // Minimum altitude for MEO
#define MEO_MAX 35786.0; // Maximum altitude for MEO

#define GEO_MIN 35786.0; // Minimum altitude for GEO
#define GEO_MAX 42164.0; // Maximum altitude for GEO


// TODO At some point, migrate internal computation/data structures to Eigen::Vector instead

class State 
{

public:

    // Constructors 
    State(std::vector<double>& initial_state);
    // Constructor initializing the vector with a size
    State(size_t size);
    // Copy constructor
    State(const State& other);
    // Constructor with initializer list
    State(std::initializer_list<double> initial_state);
    // Constructor initializing with an Eigen::VectorXd
    State(const Eigen::VectorXd& initial_state);


    // Overload [] operator for element access
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    // Overload * operator for scalar multiplication
    State operator*(double scalar);
    // Overload + operator for element-wise addition
    State operator+(const State& other) const;
    // Overload - operator for element-wise substraction
    State operator-(const State& other) const;

    // Overload << to output the state
    friend std::ostream& operator<<(std::ostream& os, const State& state);

    size_t size() const;


private:
    std::vector<double> data;
    std::size_t size;

    // Helper function to convert Eigen::VectorXd to std::vector<double>
    std::vector<double> EigenToVector(const Eigen::VectorXd& eigen_vector);

};



class State4D_ECI : public State 
{
public:
    // Constructor initializing with a 4D vector
    State4D_ECI(std::vector<double>& initial_state);
    // Constructor with initializer list
    State4D_ECI(std::initializer_list<double> initial_state);
    // Constructor initializing with zeros
    State4D_ECI();
};




class State6D : public State 
{
public:
    // Constructor initializing with a 6D vector
    State6D(std::vector<double>& initial_state);

    // Constructor with initializer list
    State6D(std::initializer_list<double> initial_state);

    // Constructor initializing with zeros
    State6D();

};



// TODO 6D ECI related checks (reasonable values for position)

class State6D_ECI : public State 
{
public:
    // Constructor initializing with a 6D vector
    State6D_ECI(std::vector<double>& initial_state);

    // Constructor with initializer list
    State6D_ECI(std::initializer_list<double> initial_state);

    // Constructor initializing with zeros
    State6D_ECI();

};




class State6D_OE : public State 
{
public:
    // Constructor initializing with a 6D vector
    State6D_OE(std::vector<double>& initial_state);

    // TODO same with initializer_list 

    // Static to be used throughout the codebase
    static void CheckOE(std::vector<double>& orbital_elements);
    
private:
    
};



class Control : public State 
{
public:

    Control(std::vector<double>& initial_state);
    Control(std::initializer_list<double> initial_state);
    Control(size_t size);
    // Constructor initializing with an Eigen::VectorXd
    Control(const Eigen::VectorXd& initial_state);

};


class Control2D : public Control
{
public:
    // Constructor initializing with a 2D vector
    Control2D(std::vector<double>& initial_state);

    // Constructor with initializer list
    Control2D(std::initializer_list<double> initial_state);

    // Constructor initializing with zeros
    Control2D();
};


class Control3D : public Control 
{
public:
    // Constructor initializing with a 3D vector
    Control3D(std::vector<double>& initial_state);

    // Constructor with initializer list
    Control3D(std::initializer_list<double> initial_state);

    // Constructor initializing with zeros
    Control3D();

};
#endif 