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
State convertArrayToState(const double* array) {
    // Ensure the input array has size 3
    State state;
    for (int i = 0; i < state.rows(); ++i) {
        state(i) = array[i];
    }
    return state;
}

static void plannerRRTStar(
    double *start_state,
    double *goal_state,
    double ***plan,
    int *planlength)
{	
	State start_state_vec = convertArrayToState(start_state);
	State goal_state_vec = convertArrayToState(goal_state);

    std::cout << std::endl << "******RRT-STAR PLANNER*****" << std::endl;
	RRT_Star_Planner planner_rrt_star = RRT_Star_Planner(start_state_vec, 
														 goal_state_vec, 
														 plan, planlength);

	std::cout << "----BUILDING TREE----" << std::endl;
	planner_rrt_star.FindPath(start_state_vec, goal_state_vec);
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
	double* startPos = doubleArrayFromString(argv[1]);
	double* goalPos = doubleArrayFromString(argv[2]);

	string outputFile = argv[3];

	State startPos_vec = convertArrayToState(startPos);
	State goalPos_vec = convertArrayToState(goalPos);

	std::cout << "Start ";
	print_config(startPos_vec);
	std::cout << "Goal ";
	print_config(goalPos_vec);

	const int numOfDOFs = startPos_vec.rows();

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    plannerRRTStar(startPos, goalPos, &plan, &planlength);

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