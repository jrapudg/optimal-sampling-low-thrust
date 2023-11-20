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
std::vector<double> ForwadSim(std::vector<double>& q_next, std::vector<double>& q_current, std::vector<double>& q_rand, double dt);

double GetTrajectoryCost(const std::vector<double>& current_state, const std::vector<double>& new_state);

double circular_distance(double angle1, double angle2);

double config_distance(const std::vector<double>& a, const std::vector<double>& b);

double calculate_norm(const std::vector<double>& config);

// Function to generate n uniformly spaced points between start and end configurations
std::vector<std::vector<double>> generate_uniform_interpolation(const std::vector<double>& start_config, const std::vector<double>& end_config, int n);

std::vector<std::vector<double>>  generate_epsilon_interpolation(const std::vector<double>& start_config, const std::vector<double>& end_config, int n, double epsilon);

bool are_configs_equal(const std::vector<double>& config1, const std::vector<double>& config2);

bool are_configs_close(const std::vector<double>& config1, const std::vector<double>& config2, double min_distance);

void print_config(const std::vector<double>& config);

#endif // UTILS_H