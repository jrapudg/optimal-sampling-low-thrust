#include "utils.hpp"

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
bool are_configs_close(const std::vector<double>& config1, const std::vector<double>& config2, double min_distance) {
    if (config1.size() != config2.size()) {
        return false;  // Configurations must be of the same size
    }

    if (config_distance(config1, config2) <= min_distance) {
        return true;
    }
    return false;
}

double GetTrajectoryCost(const std::vector<double>& state, const std::vector<double>& control){
	return config_distance(state, control);
}

double circular_distance(double angle1, double angle2) {
	// Function to calculate the circular distance between two angles in radians
    double diff = abs(angle2 - angle1);
    return MIN(diff, abs(2*M_PI - diff));
}

double config_distance(const std::vector<double>& a, const std::vector<double>& b) {
    // It's good practice to check that the sizes of the vectors are the same
    if (a.size() != b.size()) {
        throw std::invalid_argument("Vectors must be of the same size.");
    }

    double dist = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        dist += circular_distance(a[i], b[i]);
    }
    return dist;
}

double calculate_norm(const std::vector<double>& config) {
    double norm = 0.0;

    for (size_t i = 0; i < config.size(); i++) {
        norm += config[i] * config[i];
    }
    return std::sqrt(norm);
}

bool are_configs_equal(const std::vector<double>& config1, const std::vector<double>& config2) {
    if (config1.size() != config2.size()) {
        return false;  // Vectors are of different sizes
    }

    for (size_t i = 0; i < config1.size(); i++) {
        if (config1[i] != config2[i]) {
            return false; // At least one element is different
        }
    }
    return true; // All elements are equal
}

void print_config(const std::vector<double>& config) {
    std::cout << "Config (";
    for (size_t j = 0; j < config.size(); j++) {
        std::cout << config[j];
        if (j < config.size() - 1) {
            std::cout << ", ";  // Add comma between elements, but not after the last one
        }
    }
    std::cout << ")" << std::endl;
}