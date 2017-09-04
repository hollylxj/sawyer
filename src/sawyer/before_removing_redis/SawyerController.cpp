/*
 * SawyerController.h
 *
 *  Created on: Sept 1, 2017
 *      Author: Holly Liang
 */

#include "SawyerController.h"

using namespace std;

volatile bool SawyerController::runloop = true;

SawyerController::SawyerController(const std::string& robot_file){
    
    this->world_file = "";
    this->robot_file = robot_file;
    this->robot_name = "sawyer";
    
    // Start redis client
    // Make sure redis-server is running at localhost with default port 6379
    HiredisServerInfo info;
    info.hostname_ = "127.0.0.1";
    info.port_ = 6379;
    info.timeout_ = { 1, 500000 }; // 1.5 seconds
    redis_client = RedisClient();
    redis_client.serverIs(info);
    
    // Load robot
    robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
    robot->updateModel();
    dof = robot->dof();
    
    // Create a loop timer
    const double control_freq = 1000;
    timer.setLoopFrequency(control_freq);   // 1 KHz
    // timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
    timer.setCtrlCHandler(SawyerController::stop);    // exit while loop on ctrl-c
    timer.initializeTimer(1e6); // 1 ms pause before starting loop
    
    //Initialize joint Vectors
    q_err = Eigen::VectorXd(dof);
    g = Eigen::VectorXd(dof);
    command_torques = Eigen::VectorXd(dof);
    
    // Initialize PID parameters
    kp_pos = 50;//100;
    kv_pos = 35;
    kp_ori = 50;//100;
    kv_ori = 50;
    kp_joint = 15;//30;
    kv_joint = 20;
    kMaxVelocity = 0.3;
}

SawyerController::~SawyerController (){delete robot;}

Eigen::Vector3d SawyerController::getXDes(){
    return x_des;
}

void SawyerController::setXDes(Eigen::Vector3d x_des){
    this->x_des = x_des;
}

void SawyerController::setOriDes(Eigen::Vector3d ori_des){
    fixOri = false;
    this->ori_des = ori_des;
}

void SawyerController::maintainOri(){
    fixOri = true;
}

void SawyerController::setPIDParams(int kp_pos, int kv_pos, int kp_ori, int kv_ori, int kp_joint, int kv_joint){
    this->kp_pos = kp_pos;
    this->kv_pos = kv_pos;
    this->kp_ori = kp_ori;
    this->kv_ori = kv_ori;
    this->kp_joint = kp_joint;
    this->kv_joint = kv_joint;
}

void SawyerController::setKMaxVelocity(double kMaxVelocity){
    this->kMaxVelocity = kMaxVelocity;
}

void SawyerController::readFromRedis(){
    string q_isReady;
    string redis_buf;
    
    if(DEBUG){
        cout << "DEBUG: Waiting to read Q Value " << endl;
    }
    while(q_isReady != "Ready to Read"){
        try{
            q_isReady = redis_client.get(Q_READY);
        } catch (...){
            continue;
        }
    }
    
    if(DEBUG){
        cout << "DEBUG: Q Value Ready " << endl;
    }
    
    // read from Redis
    while(true){
        try{
            //joint sensor values
            redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
            redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
        
            //desired position and orientation values
            redis_client.getEigenMatrixDerivedString(EE_DES_POSITION_KEY, x_des);
            redis_client.getEigenMatrixDerivedString(EE_DES_ORIENTATION_KEY, ori_des);
            break;
        } catch (...) {
            if(DEBUG){
                cout << "DEBUG: Fail to read from Redis" << endl;
            }
            continue;
        }
    }
    
    // tell pybullet that Q value has been read
    redis_client.set(Q_READY, "Ready to Write");
    
    // Update the model
    robot->updateModel();
}

void SawyerController::writeToRedis(){
    string torque_isReady;
    
    // Wait until previous torque has been read by pybullet
    if(DEBUG){
        cout << "DEBUG: Waiting to write Torque value " << endl;
    }
    while(torque_isReady != "Ready to Write"){
        try{
            torque_isReady = redis_client.get(TORQUE_READY);
        } catch (...){
            //First loop
            break;
        }
    }
    
    // Write Torque values to Redis
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    
    // Tell pybullet that new torque value is ready
    redis_client.set(TORQUE_READY, "Ready to Read");
    if(DEBUG){
        cout << "DEBUG: Torque Value Written " << endl;
    }
}

void SawyerController::calcTorque(){
    
    // Variable Declarations
    Eigen::MatrixXd J, Lambda(6,6), N(dof,dof);
    Eigen::Matrix3d R, R_des;
    Eigen::Vector3d x,dw,w, d_phi, dx, p, ddx;
    Eigen::VectorXd nullspace_damping, ddx_dw(6), F(6);//, q_des(dof);
    
    //------ Compute controller torques
    R_des = _R_des();
    robot->J(J, "right_l6", Eigen::Vector3d::Zero());
    robot->taskInertiaMatrixWithPseudoInv(Lambda, J);
    robot->gravityVector(g);
    robot->position(x, "right_l6", Eigen::Vector3d::Zero());
    robot->rotation(R, "right_l6");
    robot->linearVelocity(dx, "right_l6", Eigen::Vector3d::Zero());
    robot->angularVelocity(w, "right_l6");
    robot->orientationError( d_phi, R_des, R );
    robot->nullspaceMatrix(N, J);
    
    //Debugging prints
    if(DEBUG){
        cout << "DEBUG: q:" << robot->_q << endl;
        cout << "DEBUG: eef pos: " << x << endl;
        cout << "DEBUG: eef des pos: " << x_des << endl;
    }
    
    
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
    
}

//Protected Methods
Eigen::Matrix3d SawyerController::_R(){
    Eigen::Matrix3d R;
    robot->rotation(R, "right_l6");
    return R;
}

Eigen::Matrix3d SawyerController::_R_des(){
    Eigen::Matrix3d R_des;
    if(fixOri){
        robot->rotation(R_des, "right_l6");
    } else {
        double cad = cos(ori_des[0]);
        double sad = sin(ori_des[0]);
        double cbd = cos(ori_des[1]);
        double sbd = sin(ori_des[1]);
        double cyd = cos(ori_des[2]);
        double syd = sin(ori_des[2]);
        R_des << cad*cbd, cad*sbd*syd-sad*cyd, cad*sbd*cyd+sad*syd,
        sad*cbd, sad*sbd*syd+cad*cyd, sad*sbd*cyd-cad*syd,
        -sbd, cbd*syd, cbd*cyd;
    }
    return R_des;
}
