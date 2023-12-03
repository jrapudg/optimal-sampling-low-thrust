#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <stack>
#include <memory>
#include <chrono>
#include <limits>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

//#include "../dynamics/astrodynamics.hpp"

#include "../dynamics/pendelum.hpp"
#include "../dynamics/lqr.hpp"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
//the length of each link in the arm
#define LINKLENGTH_CELLS 10

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


using namespace Pendelum;
//using namespace Astrodynamics;
using namespace Optimal;


//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim);

double* doubleArrayFromString(string str);

bool equalDoubleArrays(double* v1, double *v2, int size);

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          HELPER FUNCTIONS                                                         //
//                                                                                                                   //
//*******************************************************************************************************************//
Pendelum::State ForwadSim(Pendelum::State& q_next, Pendelum::State& q_current, Pendelum::State& q_rand, double dt);

// INTEGRATE HERE
//double GetTrajectoryCost(const State& current_state, const State& new_state);


// INTEGRATE HERE
double config_distance(const Pendelum::State& a, const Pendelum::State& b, const MatrixS& S);

double calculate_norm(const Pendelum::State& config);

bool are_configs_equal(const Pendelum::State& config1, const Pendelum::State& config2);

bool are_configs_close(const Pendelum::State& config1, const Pendelum::State& config2, const MatrixS& S, double min_distance);

void print_config(const Pendelum::State& config);

#endif // UTILS_H