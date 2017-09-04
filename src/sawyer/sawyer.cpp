#include "SawyerController.h"

static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
//static string world_file = "";
static string robot_file = "";
//static string robot_name = "sawyer";

const double PI = 3.141592653589793238463;

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {
    
    // Parse command line and set redis keys
    parseCommandline(argc, argv);
    
    
    //cout << "Loading URDF world model file: " << world_file << endl;
    
    SawyerController* sc = new SawyerController(robot_file);
    
    //Eigen::VectorXd q_initial = Eigen::VectorXd::Zero(dof); // Desired initial joint position
    
    
    //Eigen::Vector3d x_des;
    //Eigen::Vector3d ori_des;    // desired orientation (alpha, beta, gamma)
    
    //x_des << 0.5, 0, 0.8;
    
    //try{redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);}catch (...){continue;}
    
    //q_des = robot->_q;
    //q_des = q_initial;
    
    
    //Set desired rotation in R_des
    //ori_des << 0.0, PI/3, 0.0;
    //sc->setOriDes(ori_des);
    
    //sc->maintainOri();   //Maintain current rotation
    
    while ( SawyerController::getRunloop() ) {
        
        sc->readFromRedis();
        sc->calcTorque();
        sc->writeToRedis();
        
        //To include breakpoint each loop
        //string dummy;
        //cin >> dummy;
    }
    
    //command_torques.setZero();
    //redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    return 0;
}


//------------------------------------------------------------------------------
void parseCommandline(int argc, char** argv) {
	if (argc != 2) {
		cout << "Usage: sawyer <path-to-sawyer.urdf>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	//world_file = string(argv[1]);
	// argument 2: <path-to-robot.urdf>
	robot_file = string(argv[1]);
	// argument 3: <robot-name>
	//robot_name = string(argv[2]);
}
