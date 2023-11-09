/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include "planner.hpp"
//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength,
			int& numberOfNodes)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

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
    std::cout << std::endl << "******RRT-STAR PLANNER*****" << std::endl;
	RRT_Star_Planner planner_rrt_star = RRT_Star_Planner(armstart_anglesV_rad, 
														 armgoal_anglesV_rad, 
														 numofDOFs, map, x_size, y_size,
														 plan, planlength);

	std::cout << "----BUILDING TREE----" << std::endl;
	planner_rrt_star.FindPath();
	std::cout << "Graph Nodes: " << planner_rrt_star.tree.list.size() << std::endl;
	if (!planner_rrt_star.path_found){
		std::cout << "RESULT -> PATH NOT FOUND WITH RRT-STAR" << std::endl;
		planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength, numberOfNodes);
		std::cout << "Return interpolation" << std::endl;
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

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
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
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength, numberOfNodes);
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