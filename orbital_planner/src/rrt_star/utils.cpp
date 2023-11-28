#include "utils.hpp"

using namespace Astrodynamics;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}

double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          HELPER FUNCTIONS                                                         //
//                                                                                                                   //
//*******************************************************************************************************************//
bool are_configs_equal(const State& config1, const State& config2) {
    return config1.isApprox(config2, 0.0);  // Using Eigen's isApprox function with zero tolerance for exact equality.
}

bool are_configs_close(const State& config1, const State& config2, double min_distance) {
    return config_distance(config1, config2) <= min_distance;  // Simplified to a single line.
}

double GetTrajectoryCost(const State& state, const State& control){
	return config_distance(state, control);
}

double circular_distance(double angle1, double angle2) {
	// Function to calculate the circular distance between two angles in radians
    double diff = abs(angle2 - angle1);
    return MIN(diff, abs(2*M_PI - diff));
};

double config_distance(const State& a, const State& b) {
    // Using rows() to determine the size of the matrix.
    int size = a.rows();

    double dist = 0;
    for (int i = 0; i < size; ++i) {
        dist += circular_distance(a(i), b(i));
    }
    return dist;
};

double calculate_norm(const State& config) {
    return config.norm();  // Eigen's built-in function to calculate the Euclidean norm.
};

void print_config(const State& config) {
    std::cout << "Config (";
    for (int i = 0; i < config.rows(); ++i) {
        std::cout << config(i);
        if (i < config.rows() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << ")" << std::endl;
}