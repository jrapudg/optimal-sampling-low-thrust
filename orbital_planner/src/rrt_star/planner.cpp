/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "planner.hpp"

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//
std::vector<double> convertArrayToVector(const double* array, int size) {
    // Construct a vector from the array elements
    return std::vector<double>(array, array + size);
}

static void plannerRRTStar(
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{	
	std::vector<double> armstart_anglesV_rad_vec = convertArrayToVector(armstart_anglesV_rad, numofDOFs);
	std::vector<double> armgoal_anglesV_rad_vec = convertArrayToVector(armgoal_anglesV_rad, numofDOFs);

    std::cout << std::endl << "******RRT-STAR PLANNER*****" << std::endl;
	RRT_Star_Planner planner_rrt_star = RRT_Star_Planner(armstart_anglesV_rad_vec, 
														 armgoal_anglesV_rad_vec, 
														 numofDOFs, plan, planlength);

	std::cout << "----BUILDING TREE----" << std::endl;
	planner_rrt_star.FindPath(armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec);
	std::cout << "Graph Nodes: " << planner_rrt_star.tree.list.size() << std::endl;
	if (!planner_rrt_star.path_found){
		std::cout << "RESULT -> PATH NOT FOUND WITH RRT-STAR" << std::endl;
	}
	std::cout << "*******PLAN DONE*******" << std::endl << std::endl;

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

int main(int argc, char** argv) {
	const int numOfDOFs = std::stoi(argv[3]);
	double* startPos = doubleArrayFromString(argv[1]);
	double* goalPos = doubleArrayFromString(argv[2]);
	string outputFile = argv[4];

	std::vector<double> startPos_vec = convertArrayToVector(startPos, numOfDOFs);
	std::vector<double> goalPos_vec = convertArrayToVector(goalPos, numOfDOFs);

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    plannerRRTStar(startPos, goalPos, numOfDOFs, &plan, &planlength);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	// * Saves the solution to output file
	// * Do not modify the output log file output format as it is required for visualization
	// * and for grading.

	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}