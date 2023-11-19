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

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath);

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim);

double* doubleArrayFromString(string str);

bool equalDoubleArrays(double* v1, double *v2, int size);

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);

void get_current_point(bresenham_param_t *params, int *x, int *y);

int get_next_point(bresenham_param_t *params);

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size);

int IsValidArmConfiguration(std::vector<double>& angles, double* map,
							int x_size, int y_size);

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          HELPER FUNCTIONS                                                         //
//                                                                                                                   //
//*******************************************************************************************************************//
double get_cost(const std::vector<double>& state, const std::vector<double>& control);

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