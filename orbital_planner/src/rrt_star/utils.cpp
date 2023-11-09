#include "utils.hpp"

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

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

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          HELPER FUNCTIONS                                                         //
//                                                                                                                   //
//*******************************************************************************************************************//
/*
double circular_distance(double angle1, double angle2) {
	// Function to calculate the circular distance between two angles in radians
    double diff = fmod(angle2 - angle1 + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
    return fabs(diff);
};

double config_distance(const double* a, const double* b, int DOF) {
	double dist = 0;
	for (size_t i = 0; i < DOF; ++i) {
		dist += std::pow(circular_distance(a[i], b[i]), 2);
	}
	return std::sqrt(dist);
};
*/

double circular_distance(double angle1, double angle2) {
	// Function to calculate the circular distance between two angles in radians
    double diff = abs(angle2 - angle1);
    return MIN(diff, abs(2*M_PI - diff));
};

double config_distance(const double* a, const double* b, int DOF) {
	double dist = 0;
	for (size_t i = 0; i < DOF; ++i) {
		dist += circular_distance(a[i], b[i]);
	}
	return dist;
};

double calculate_norm(const double* config, int DOF) {
    double norm = 0.0;
    
    for (int i = 0; i < DOF; i++) {
        norm += config[i] * config[i];
    }
    
    return std::sqrt(norm);
};

// Function to generate n uniformly spaced points between start and end configurations
std::vector<double*> generate_uniform_interpolation(const double* start_config, const double* end_config, int n, int DOF) {
    std::vector<double*> interpolated_points;

    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1); // Interpolation parameter [0, 1]

        double* interpolated_config = new double[DOF];

        // Linear interpolation for each degree of freedom
        for (int j = 0; j < DOF; ++j) {
            interpolated_config[j] = start_config[j] + t * (end_config[j] - start_config[j]);
        }
        interpolated_points.push_back(interpolated_config);
    }

    return interpolated_points;
};

std::vector<double*>  generate_epsilon_interpolation(const double* start_config, const double* end_config, int n, int DOF, double epsilon) {
    double distance = config_distance(start_config, end_config, DOF);
	std::vector<double*> interpolated_points;

    if (distance <= epsilon) {
        interpolated_points = generate_uniform_interpolation(start_config, end_config, n, DOF);
    } else {
	
        // Interpolate towards end_config an epsilon distance
		double epsilon_config[DOF];
        for (int i = 0; i < DOF; i++) {
            double diff = end_config[i] - start_config[i];
            double step = epsilon * diff / distance;
            epsilon_config[i] = start_config[i] + step;
        }
        interpolated_points = generate_uniform_interpolation(start_config, epsilon_config, n, DOF);
    }
	return interpolated_points;
};

// Function to deallocate memory for the dynamically allocated arrays
void deallocate_interpolated_points(std::vector<double*>& points) {
    for (double* point : points) {
        delete[] point; // Deallocate the dynamically allocated array
    }
    points.clear(); // Clear the vector to release the pointers
};

bool are_configs_equal(const double* config1, const double* config2, int dof){
    for (int i = 0; i < dof; i++) {
        if (config1[i] != config2[i]) {
            return false; // At least one element is different
        }
    }
    return true; // All elements are equal
};

bool are_configs_close(const double* config1, const double* config2, int dof, double min_distance){
	if (config_distance(config1, config2, dof) <= min_distance){
		return true;
	}
	return false;
};

void print_config(double* config, int DOF){
	std::cout << "Config (";
	for (int j = 0; j < DOF; j++){
		std::cout << config[j] << " ";
	}
	std::cout << ")" << std::endl;
};