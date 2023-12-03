#include "planner.hpp"

using namespace Astrodynamics;

State convertArrayToState(const double* array) {
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


}

int main(int argc, char** argv) 
{

	double* start_d = doubleArrayFromString(argv[1]);
	double* goal_d = doubleArrayFromString(argv[2]);
	string outputFile = argv[3];


	State start_state = convertArrayToState(start_d);
	State goal_state = convertArrayToState(goal_d);
	Print(start_state, "Start");
	Print(goal_state, "Goal");


	const int n_dofs = start_state.rows();
	double** plan = NULL;
	int plan_length = 0;


    std::cout << std::endl << "******WELCOME TO LQR-RRT* MY FRIEND*****" << std::endl;
	RRT_Star_Planner planner_rrt_star = RRT_Star_Planner(start_state, goal_state, &plan, &plan_length);
														 
														 
	std::cout << "---- Starting LQR-RRT* ----" << std::endl;
	planner_rrt_star.FindPath(start_state, goal_state);




	std::cout << "Graph Nodes: " << planner_rrt_star.tree.list.size() << std::endl;
	if (!planner_rrt_star.path_found){
		std::cout << "RESULT -> PATH NOT FOUND WITH RRT-STAR" << std::endl;
	}
	std::cout << "*******PLAN DONE*******" << std::endl << std::endl;





    // Your solution's path should start with start_d and end with goal_d
    if (!equalDoubleArrays(plan[0], start_d, n_dofs) || 
    	!equalDoubleArrays(plan[plan_length-1], goal_d, n_dofs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	// * Saves the solution to output file

	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < plan_length; ++i) {
		for (int k = 0; k < n_dofs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}