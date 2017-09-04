/*
 * SawyerController.h
 *
 *  Created on: Sept 1, 2017
 *      Author: Holly Liang
 */

#ifndef SAWYERCONTROLLER_H_
#define SAWYERCONTROLLER_H_

#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>

using namespace std;

static string robot_name = "sawyer";

static unsigned long long controller_counter = 0;

// Redis keys:
// - write:
static std::string JOINT_TORQUES_COMMANDED_KEY = "py::robot::" + robot_name + "::actuators::fgc";

// - read:
static std::string JOINT_ANGLES_KEY       = "py::robot::" + robot_name + "::sensors::q";
static std::string JOINT_VELOCITIES_KEY   = "py::robot::" + robot_name + "::sensors::dq";
static std::string EE_POSITION_KEY        = "py::robot::" + robot_name + "::tasks::ee_pos";
static std::string EE_DES_POSITION_KEY    = "py::robot::" + robot_name + "::tasks::ee_pos_des";
static std::string EE_DES_ORIENTATION_KEY = "py::robot::" + robot_name + "::tasks::ee_ori_des";
static std::string Q_READY                = "py::robot::" + robot_name + "::tasks::q_flag";
static std::string TORQUE_READY           = "py::robot::" + robot_name + "::tasks::torque_flag";
static std::string TIMESTAMP_KEY          = "py::robot::" + robot_name + "::timestamp";

const bool DEBUG = true;

class SawyerController {
    
    //Properties
private:
    // Location of URDF files specifying world and robot information
    string world_file = "";
    string robot_file = "";
    string robot_name = "sawyer";
    
    static volatile bool runloop;
    
    RedisClient redis_client;
    Model::ModelInterface* robot;  //Virtual Sawyer robot object
    int dof;    //Degree of Freedom of Sawyer robot
    bool fixOri;    //Maintain current orientation
    LoopTimer timer;
    Eigen::Vector3d x_des;  //Desired end-effector position
    Eigen::Vector3d ori_des; //Desired end-effector orientation
    Eigen::VectorXd q_err;  //q_err = q_des - q
    Eigen::VectorXd g;      //gravity vector in joint space
    Eigen::VectorXd command_torques;
    
    //PID parameters
    int kp_pos;
    int kv_pos;
    int kp_ori;
    int kv_ori;
    int kp_joint;
    int kv_joint;
    double kMaxVelocity;
    
public:
    //Constructor
    SawyerController(const std::string& robot_file);
    
    //Destructor
    ~SawyerController();
    
    //Methods
    Eigen::Vector3d getXDes();
    void setXDes(Eigen::Vector3d x_des);
    void setOriDes(Eigen::Vector3d ori_des);
    void maintainOri();
    void setPIDParams(int kp_pos, int kv_pos, int kp_ori, int kv_ori, int kp_joint, int kv_joint);
    void setKMaxVelocity(double kMaxVelocity);
    
    static bool getRunloop() { return runloop; }
    static void stop(int){ runloop = false; }
    void readFromRedis();
    void writeToRedis();
    void calcTorque();
    
    std::vector<double> testEigen(const std::vector<double> x){

        Eigen::Vector3d y(0.0,0.0,0.0);
        std::vector<double> z;
        
        for(int i=0; i<3; i++){
            y[i]=x[i];
            z.push_back(x[i]*2);
        }
        cout << "y contains:" << y << endl;
        
        return z;
    }
    
protected:
    Eigen::Matrix3d _R();
    Eigen::Matrix3d _R_des();
};

#endif /* SAWYERCONTROLLER_H_ */
