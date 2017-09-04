#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>
static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

// Location of URDF files specifying world and robot information
//static string world_file = "";
static string robot_file = "";
static string robot_name = "sawyer";

static unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "";
static std::string PY_JOINT_TORQUES_COMMANDED_KEY = "";

// - read:
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string PY_JOINT_ANGLES_KEY  = "";
static std::string PY_JOINT_VELOCITIES_KEY = "";
static std::string TIMESTAMP_KEY = "";
static std::string KP_POSITION_KEY = "";
static std::string KV_POSITION_KEY = "";
static std::string KP_ORIENTATION_KEY = "";
static std::string KV_ORIENTATION_KEY = "";
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";
static std::string EE_POSITION_KEY = "";
static std::string EE_DES_POSITION_KEY = "";
static std::string PY_EE_POSITION_KEY = "";
static std::string PY_EE_DES_POSITION_KEY = "";
static std::string PY_EE_DES_ORIENTATION_KEY = "";
static std::string PY_Q_READY = "";
static std::string PY_TORQUE_READY = "";

const double PI = 3.141592653589793238463;

// Function to parse command line arguments
void parseCommandline(int argc, char** argv);

int main(int argc, char** argv) {

	// Parse command line and set redis keys
	parseCommandline(argc, argv);
	JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
    PY_JOINT_TORQUES_COMMANDED_KEY = "py::robot::" + robot_name + "::actuators::fgc";
	JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
	JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
    PY_JOINT_ANGLES_KEY         = "py::robot::" + robot_name + "::sensors::q";
    PY_JOINT_VELOCITIES_KEY     = "py::robot::" + robot_name + "::sensors::dq";
	TIMESTAMP_KEY               = "cs225a::robot::" + robot_name + "::timestamp";
	KP_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kp_pos";
	KV_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kv_pos";
	KP_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kp_ori";
	KV_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kv_ori";
	KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
	KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
	KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
	KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";
	EE_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::ee_pos";
    EE_DES_POSITION_KEY         = "cs225a::robot::" + robot_name + "::tasks::ee_pos_des";
    PY_EE_POSITION_KEY          = "py::robot::" + robot_name + "::tasks::ee_pos";
    PY_EE_DES_POSITION_KEY      = "py::robot::" + robot_name + "::tasks::ee_pos_des";
    PY_EE_DES_ORIENTATION_KEY   = "py::robot::" + robot_name + "::tasks::ee_ori_des";
    PY_Q_READY              = "py::robot::" + robot_name + "::tasks::q_flag";
    PY_TORQUE_READY              = "py::robot::" + robot_name + "::tasks::torque_flag";

	//cout << "Loading URDF world model file: " << world_file << endl;

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// Load robot
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();
	const int dof = robot->dof();
	// Create a loop timer
	const double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1e6); // 1 ms pause before starting loop
	
	Eigen::VectorXd command_torques(dof);
	Eigen::VectorXd q_err(dof), g(dof);
	Eigen::VectorXd q_initial = Eigen::VectorXd::Zero(dof); // Desired initial joint position

	Eigen::MatrixXd J, Lambda(6,6), N(dof,dof);
	Eigen::Matrix3d R, R_des;
	Eigen::Vector3d x, x_des, dx, p, w, d_phi, ddx, dw;
    Eigen::Vector3d ori_des;    // desired orientation (alpha, beta, gamma)
	Eigen::VectorXd nullspace_damping, q_des(dof), ddx_dw(6), F(6);
	Eigen::Vector3d x_initial;
	x_des << 0.5, 0, 0.8;

    //try{redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);}catch (...){continue;}

	//q_des = robot->_q;
	q_des = q_initial;	

    int kp_pos = 50;//100;
	int kv_pos = 35;
	int kp_ori = 50;//100;
	int kv_ori = 50;
	int kv_joint = 20;
    int kp_joint = 15;//30;
	double kMaxVelocity = 0.3;
	string redis_buf;
    string q_isReady;
    string torque_isReady;
	double t_curr = 0;
	robot->position(x_initial, "right_l6", Eigen::Vector3d::Zero());
	
    //Set desired rotation in R_des
    //robot->rotation(R_des, "right_l6");   //Maintain current rotation
    ori_des << 0.0, PI/3, 0.0;
    double cad = cos(ori_des[0]);
    double sad = sin(ori_des[0]);
    double cbd = cos(ori_des[1]);
    double sbd = sin(ori_des[1]);
    double cyd = cos(ori_des[2]);
    double syd = sin(ori_des[2]);
    R_des << cad*cbd, cad*sbd*syd-sad*cyd, cad*sbd*cyd+sad*syd,
             sad*cbd, sad*sbd*syd+cad*cyd, sad*sbd*cyd-cad*syd,
             -sbd, cbd*syd, cbd*cyd;
    
    
    
    
    
    
	while (runloop) {
        
        cout << "DEBUG: Waiting to read Q Value " << endl;
        while(q_isReady != "Ready to Read"){
            try{
                q_isReady = redis_client.get(PY_Q_READY);
            } catch (...){
                continue;
            }
        }
        cout << "DEBUG: Q Value Ready " << endl;
        
		// Get current simulation timestamp from Redis
        try{redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);}catch (...){continue;}

		if (redis_buf.length() != 0)
		{
			t_curr = stod(redis_buf);
		}

		// read from Redis current sensor values
        //redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
        //redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
		try{redis_client.getEigenMatrixDerivedString(PY_JOINT_ANGLES_KEY, robot->_q);}catch (...){continue;}
		try{redis_client.getEigenMatrixDerivedString(PY_JOINT_VELOCITIES_KEY, robot->_dq);}catch (...){continue;}

        // read from Redis desired position and orientation values
        try{redis_client.getEigenMatrixDerivedString(PY_EE_DES_POSITION_KEY, x_des);}catch (...){continue;}
        try{redis_client.getEigenMatrixDerivedString(PY_EE_DES_ORIENTATION_KEY, ori_des);}catch (...){continue;}
        
        // tell pybullet that Q value has been read
        redis_client.set(PY_Q_READY, "Ready to Write");
        
        // calculate R_des
        cad = cos(ori_des[0]);
        sad = sin(ori_des[0]);
        cbd = cos(ori_des[1]);
        sbd = sin(ori_des[1]);
        cyd = cos(ori_des[2]);
        syd = sin(ori_des[2]);
        R_des << cad*cbd, cad*sbd*syd-sad*cyd, cad*sbd*cyd+sad*syd,
        sad*cbd, sad*sbd*syd+cad*cyd, sad*sbd*cyd-cad*syd,
        -sbd, cbd*syd, cbd*cyd;
        //redis_client.getEigenMatrixDerivedString(PY_EE_POSITION_KEY, x);
        
		// Update the model
		robot->updateModel();
		robot->gravityVector(g);
		//------ Compute controller torques
		robot->J(J, "right_l6", Eigen::Vector3d::Zero());
		robot->taskInertiaMatrixWithPseudoInv(Lambda, J);
		robot->gravityVector(g);
		robot->position(x, "right_l6", Eigen::Vector3d::Zero());
		robot->rotation(R, "right_l6");
		robot->linearVelocity(dx, "right_l6", Eigen::Vector3d::Zero());
		robot->angularVelocity(w, "right_l6");
		robot->orientationError(d_phi, R_des, R);
		robot->nullspaceMatrix(N, J);

        //Debugging prints
        cout << "DEBUG: q:" << robot->_q << endl;
        cout << "DEBUG: eef pos: " << x << endl;
        cout << "DEBUG: eef des pos: " << x_des << endl;
        
		//x_des << -0.2, 0.2 * sin(0.1*M_PI*t_curr) - 0.1, 0.2 + 0.2 * cos(0.1*M_PI*t_curr);
		//x_des += x_initial;

		// Send end effector position for trajectory visualization
        redis_client.setEigenMatrixDerivedString(EE_DES_POSITION_KEY, x_des);
        redis_client.setEigenMatrixDerivedString(EE_POSITION_KEY, x);

		// Orientation and position controller with pose and damping in the nullspace
		dw = -kp_ori * d_phi - kv_ori * w;

		// Velocity Control
		Eigen::Vector3d x_err = x - x_des;
		Eigen::Vector3d dx_des = -(kp_pos / kv_pos) * x_err;
		double v = kMaxVelocity / dx_des.norm();
        
        // speed saturation
		//if (v > 1) v = 1;
		Eigen::Vector3d dx_err = dx - v * dx_des;
		ddx = -kv_pos * dx_err;

		ddx_dw << dw, ddx;
		F = Lambda * ddx_dw;
		//nullspace_damping = N.transpose() * robot->_M * (kp_joint * (q_des - robot->_q) - kv_joint * robot->_dq);
        nullspace_damping = N.transpose() * robot->_M * (-kv_joint * robot->_dq);

		command_torques = J.transpose() * F + nullspace_damping + g; // Don't forget to add or remove gravity for real robot
		// command_torques = robot->_M * (kp_joint * (q_des - robot->_q) - kv_joint * robot->_dq); // Joint Control Law

        // Wait until previous torque has been read by pybullet
        cout << "DEBUG: Waiting to write Torque value " << endl;
        while(torque_isReady != "Ready to Write"){
            try{
                torque_isReady = redis_client.get(PY_TORQUE_READY);
            } catch (...){
                //First loop
                break;
            }
        }
        
		//redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
        redis_client.setEigenMatrixDerivedString(PY_JOINT_TORQUES_COMMANDED_KEY, command_torques);
        
        // Tell pybullet that new torque value is ready
        redis_client.set(PY_TORQUE_READY, "Ready to Read");
        cout << "DEBUG: Torque Value Written " << endl;
        
        //To include breakpoint each loop
        //string dummy;
        //cin >> dummy;
	}
	
	command_torques.setZero();
	redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
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
