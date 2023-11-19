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
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength,
	int& numberOfNodes)
{	
	std::vector<double> armstart_anglesV_rad_vec = convertArrayToVector(armstart_anglesV_rad, numofDOFs);
	std::vector<double> armgoal_anglesV_rad_vec = convertArrayToVector(armgoal_anglesV_rad, numofDOFs);

    std::cout << std::endl << "******RRT-STAR PLANNER*****" << std::endl;
	RRT_Star_Planner planner_rrt_star = RRT_Star_Planner(armstart_anglesV_rad_vec, 
														 armgoal_anglesV_rad_vec, 
														 numofDOFs, map, x_size, y_size,
														 plan, planlength);

	std::cout << "----BUILDING TREE----" << std::endl;
	planner_rrt_star.FindPath(armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec);
	std::cout << "Graph Nodes: " << planner_rrt_star.tree.list.size() << std::endl;
	if (!planner_rrt_star.path_found){
		std::cout << "RESULT -> PATH NOT FOUND WITH RRT-STAR" << std::endl;
	}
	std::cout << "*******PLAN DONE*******" << std::endl << std::endl;
	numberOfNodes = planner_rrt_star.tree.list.size();

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */


int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	int numberOfNodes;
	string outputFile = argv[6];
	string outputFileExtra = "extra.txt";

	std::vector<double> startPos_vec = convertArrayToVector(startPos, numOfDOFs);
	std::vector<double> goalPos_vec = convertArrayToVector(goalPos, numOfDOFs);

	if(!IsValidArmConfiguration(startPos_vec, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos_vec, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
    }
    else
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
    }

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
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
	std::ofstream extra_log_fstream;
	extra_log_fstream.open(outputFileExtra, std::ios::trunc); // Creates new or replaces existing file
	if (!extra_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	extra_log_fstream << numberOfNodes << endl; // Write out map name first
}