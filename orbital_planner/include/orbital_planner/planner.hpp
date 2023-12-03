#ifndef PLANNER_H
#define PLANNER_H


#include "orbital_planner/rrt_star.hpp"
#include "orbital_planner/utils.hpp"
#include "orbital_planner/astrodynamics.hpp"

#include <math.h>
#include <vector>
#include <array>

#include <string>
#include <stdexcept>

#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

#endif // PLANNER_H