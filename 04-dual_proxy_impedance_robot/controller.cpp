// This example tests robot motion control with impedance-based haptic feedback.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"
#include "../src/Logger.h"

#include <iostream>
#include <string>
#include <random>
#include <queue>
#include <Eigen/Sparse>

#define INIT            0
#define CONTROL         1

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";

// redis keys:
// robot local control loop
string JOINT_ANGLES_KEY = "sai2::HapticApplications::05::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::HapticApplications::05::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::HapticApplications::05::simviz::actuators::tau_cmd";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

// posori task
// state information
string EE_POS_KEY = "sai2::HapticApplications::05::posori::ee_pos";
string EE_ORI_KEY = "sai2::HapticApplications::05::posori::ee_ori";
string EE_FORCE_KEY = "sai2::HapticApplications::05::posori::ee_force";
string EE_MOMENT_KEY = "sai2::HapticApplications::05::posori::ee_moment";
// user parameters
string EE_POS_IN_LINK_KEY = "sai2::HapticApplications::05::posori::ee_pos_in_link";
string EE_ROT_IN_LINK_KEY = "sai2::HapticApplications::05::posori::ee_rot_in_link";
string ANGULAR_MOTION_AXIS_KEY = "sai2::HapticApplications::05::posori::angular_motion_axis";
string POSORI_WRITE_PARAMS_KEY = "sai2::HapticApplications::05::posori::write_params"; // set to 1 to write parameter changes

// dual proxy
string ROBOT_PROXY_KEY = "sai2::HapticApplications::05::dual_proxy::robot_proxy";
string ROBOT_PROXY_ROT_KEY = "sai2::HapticApplications::05::dual_proxy::robot_proxy_rot";
string HAPTIC_PROXY_KEY = "sai2::HapticApplications::05::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::HapticApplications::05::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::HapticApplications::05::dual_proxy::sigma_force";
string ROBOT_DEFAULT_ROT_KEY = "sai2::HapticApplications::05::dual_proxy::robot_default_rot";
string ROBOT_DEFAULT_POS_KEY = "sai2::HapticApplications::05::dual_proxy::robot_default_pos";

string HAPTIC_DEVICE_READY_KEY = "sai2::HapticApplications::05::dual_proxy::haptic_device_ready";
string CONTROLLER_RUNNING_KEY = "sai2::HapticApplications::05::dual_proxy::controller_running";


RedisClient redis_client;

// force space parameters
int force_space_dimension = 0;
int prev_force_space_dimension = 0;
Matrix3d sigma_force = Matrix3d::Zero();
Matrix3d sigma_motion = Matrix3d::Identity();

const double control_loop_freq = 1000.0;

// helper function for creating + initializing posori task
unique_ptr<Sai2Primitives::PosOriTask> init_posori(Sai2Model::Sai2Model* robot,
                                                    const std::string link_name,
                                                    const Eigen::Vector3d pos_in_link,
                                                    const Eigen::Matrix3d rot_in_link);

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {

    if(!flag_simulation) {
        ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
        MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
    }

    // start redis client local
    redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    Affine3d T_world_robot = Affine3d::Identity();
    T_world_robot.translation() = Vector3d(0, 0, 0);
    auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);
    int state = INIT;
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_use_interpolation_flag = true;
    joint_task->_use_velocity_saturation_flag = false;

    joint_task->_kp = 200.0;
    joint_task->_kv = 25.0;
    joint_task->_ki = 50.0;

    VectorXd q_init(dof);
//    q_init << 0, -30, 0, -130, 0, 70, 0;
//    q_init *= M_PI/180.0;
    // q_init << 0.943667,-0.76595,-2.18203,-1.72349,-0.701489,2.13662,-0.0375883;
//    joint_task->_desired_position = q_init;
    q_init = robot->_q;
    joint_task->_desired_position = q_init; // use current robot config as init config

    // posori task
    // set control link and point for posori task
    const string link_name = "end_effector";
    // const Vector3d pos_in_link = Vector3d(0.0,0.0,0.153); // TODO: get measurement from borns
    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.1);
    Matrix3d rot_in_link = Matrix3d::Identity();
    unique_ptr<Sai2Primitives::PosOriTask> posori_task = init_posori(robot, link_name, pos_in_link, rot_in_link);

    VectorXd posori_task_torques = VectorXd::Zero(dof);

    // flag for setting new posori parameters
    int posori_write_params = 0;

    // track robot's autonomous ori change during surface-surface alignment
    Matrix3d R_proxy_robot = Matrix3d::Identity();

    // normal axis for surface-surface alignment
    Vector3d angular_motion_axis = Vector3d(0.0, 0.0, 1.0);

    // dual proxy parameters and variables
    double k_vir = 250.0;
    double max_force_diff = 0.1;
    double max_force = 10.0;

    int haptic_ready = 0;

    // initialize redis values
    redis_client.setEigenMatrixJSON(EE_POS_IN_LINK_KEY, pos_in_link);
    redis_client.setEigenMatrixJSON(EE_ROT_IN_LINK_KEY, rot_in_link);
    redis_client.setEigenMatrixJSON(ANGULAR_MOTION_AXIS_KEY, angular_motion_axis);
    redis_client.set(POSORI_WRITE_PARAMS_KEY, std::to_string(posori_write_params));
    redis_client.set(HAPTIC_DEVICE_READY_KEY, std::to_string(haptic_ready));

    Vector3d robot_proxy = Vector3d::Zero();
    Matrix3d robot_proxy_rot = Matrix3d::Identity();
    Vector3d haptic_proxy = Vector3d::Zero();

    // setup redis keys to be updated with the callback
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

    // Objects to read from redis
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);
    if(!flag_simulation) {
        redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot);
        redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot);
    }

    redis_client.addEigenToReadCallback(0, ROBOT_PROXY_KEY, robot_proxy);
    redis_client.addEigenToReadCallback(0, ROBOT_PROXY_ROT_KEY, robot_proxy_rot);
    redis_client.addIntToReadCallback(0, HAPTIC_DEVICE_READY_KEY, haptic_ready);

    // user parameters
    redis_client.addEigenToReadCallback(0, EE_POS_IN_LINK_KEY, pos_in_link);
    redis_client.addEigenToReadCallback(0, EE_ROT_IN_LINK_KEY, rot_in_link);
    redis_client.addEigenToReadCallback(0, ANGULAR_MOTION_AXIS_KEY, angular_motion_axis);
    redis_client.addIntToReadCallback(0, POSORI_WRITE_PARAMS_KEY, posori_write_params);

    // Objects to write to redis
    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

    redis_client.addEigenToWriteCallback(0, HAPTIC_PROXY_KEY, haptic_proxy);
    redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
    redis_client.addIntToWriteCallback(0, FORCE_SPACE_DIMENSION_KEY, force_space_dimension);

    // setup data logging
    string folder = "../../03-dual_proxy_surface_alignment_robot/data_logging/data/";
    string filename = "data";
    auto logger = new Logging::Logger(100000, folder + filename);
    
    Vector3d log_ee_position = posori_task->_current_position;
    Vector3d log_ee_velocity = Vector3d::Zero();
    Vector3d log_robot_proxy_position = robot_proxy;
    VectorXd log_joint_angles = robot->_q;
    VectorXd log_joint_velocities = robot->_dq;
    VectorXd log_joint_command_torques = command_torques;

    Matrix3d log_ee_orientation = posori_task->_current_orientation; // TODO: add to logger

    logger->addVectorToLog(&log_ee_position, "ee_position");
    logger->addVectorToLog(&log_ee_velocity, "ee_velocity");
    logger->addVectorToLog(&log_robot_proxy_position, "robot_proxy_position");
    logger->addVectorToLog(&log_joint_angles, "joint_angles");
    logger->addVectorToLog(&log_joint_velocities, "joint_velocities");
    logger->addVectorToLog(&log_joint_command_torques, "joint_command_torques");

    logger->start();

    // write internal controller state to redis
    redis_client.addEigenToWriteCallback(0, EE_POS_KEY, log_ee_position);
    redis_client.addEigenToWriteCallback(0, EE_ORI_KEY, log_ee_orientation);

    // start particle filter thread
    runloop = true;
    redis_client.set(CONTROLLER_RUNNING_KEY,"1");

    // create a timer
    unsigned long long controller_counter = 0;
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(control_loop_freq); //Compiler en mode release
    double current_time = 0;
    double prev_time = 0;
    // double dt = 0;
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime(); //secs

    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        current_time = timer.elapsedTime() - start_time;

        // read haptic state and robot state
        redis_client.executeReadCallback(0);
        if(flag_simulation) {
            robot->updateModel();
            robot->coriolisForce(coriolis);
        }
        else {
            robot->updateKinematics();
            robot->_M = mass_from_robot;
            robot->updateInverseInertia();
            coriolis = coriolis_from_robot;
        }

        N_prec.setIdentity(dof,dof);

        posori_task->updateTaskModel(N_prec);
        N_prec = posori_task->_N;

        joint_task->updateTaskModel(N_prec);

        // update force state based on impedance


        if(state == INIT) {
            // reset posori if params changed
            if(posori_write_params == 1)
            {
                // reset posori write params flag
                posori_write_params = 0;
                redis_client.set(POSORI_WRITE_PARAMS_KEY, "0");

                // reset posori controller and reinitialize joint controller
                posori_task = init_posori(robot, link_name, pos_in_link, rot_in_link);

                joint_task->reInitializeTask();
                joint_task->_kp = 200.0;
                joint_task->_kv = 25.0;
                joint_task->_ki = 50.0;
            }

            joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if(haptic_ready && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2) {
                // Reinitialize controllers
                posori_task->reInitializeTask();
                posori_task->setFullAngularMotionControl();

                joint_task->reInitializeTask();
                joint_task->_kp = 50.0;
                joint_task->_kv = 13.0;
                joint_task->_ki = 0.0;

                state = CONTROL;
            }
        }

        else if(state == CONTROL) {
            // dual proxy
            posori_task->_sigma_force = sigma_force;
            posori_task->_sigma_position = sigma_motion;
            Vector3d robot_position = posori_task->_current_position;
            Matrix3d robot_orientation = posori_task->_current_orientation;

            // motion
            Vector3d position_proxy = robot_position + sigma_motion * (robot_proxy - robot_position);
            Matrix3d rotation_proxy = robot_proxy_rot * R_proxy_robot;

            // control
            posori_task->_desired_position = position_proxy;
            posori_task->_desired_orientation = rotation_proxy;

            try {
                posori_task->computeTorques(posori_task_torques);
            }
            catch(exception e) {
                cout << "control cycle: " << controller_counter << endl;
                cout << "error in the torque computation of posori_task:" << endl;
                cerr << e.what() << endl;
                cout << "setting torques to zero for this control cycle" << endl;
                cout << endl;
                // posori_task_torques.setZero(); // set task torques to zero, TODO: test this
            }
            joint_task->computeTorques(joint_task_torques);

            command_torques = posori_task_torques + joint_task_torques + coriolis;

            // remember values
            prev_force_space_dimension = force_space_dimension;

            // move to init state if posori params changed
            if(posori_write_params == 1)
            {
                std::cout << "Moved from CONTROL to INIT" << std::endl;
                state = INIT;
            }
        }

        // write control torques and dual proxy variables
        robot->position(haptic_proxy, link_name, pos_in_link);
        redis_client.executeWriteCallback(0);

        // update logger values
        Vector3d ee_vel = Vector3d::Zero();
        robot->linearVelocity(ee_vel, link_name, pos_in_link);
        
        log_ee_position = haptic_proxy;
        log_ee_velocity = ee_vel;
        log_robot_proxy_position = robot_proxy;
        log_joint_angles = robot->_q;
        log_joint_velocities = robot->_dq;
        log_joint_command_torques = command_torques;

        log_ee_orientation = posori_task->_current_orientation;
    
        controller_counter++;
    }

    // stop logger
    logger->stop();

    delete robot;
    delete joint_task;
    delete logger;

    //// Send zero force/torque to robot ////
    command_torques.setZero();
    redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");


    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

unique_ptr<Sai2Primitives::PosOriTask> init_posori(Sai2Model::Sai2Model* robot,
                                                    const std::string link_name,
                                                    const Eigen::Vector3d pos_in_link,
                                                    const Eigen::Matrix3d rot_in_link)
{
    unique_ptr<Sai2Primitives::PosOriTask> posori_task( new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link, rot_in_link) );
    Vector3d x_init = posori_task->_current_position;
    Matrix3d R_init = posori_task->_current_orientation;
    redis_client.setEigenMatrixJSON(ROBOT_PROXY_KEY, x_init);
    redis_client.setEigenMatrixJSON(ROBOT_PROXY_ROT_KEY, R_init);
    redis_client.setEigenMatrixJSON(HAPTIC_PROXY_KEY, x_init);
    // compute expected default rotation to send to haptic
    // robot->_q = q_init;
    // robot->updateModel();
    Matrix3d R_default = Matrix3d::Identity();
    Vector3d pos_default = Vector3d::Zero();
    robot->rotation(R_default, link_name);
    robot->position(pos_default, link_name, pos_in_link);
    redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_ROT_KEY, R_default);
    redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_POS_KEY, pos_default);

    // cout << "R default:\n" << R_default << endl;

    posori_task->_use_interpolation_flag = true;

    posori_task->_otg->setMaxLinearVelocity(0.30);
    posori_task->_otg->setMaxLinearAcceleration(1.0);
    posori_task->_otg->setMaxLinearJerk(5.0);

    posori_task->_otg->setMaxAngularVelocity(M_PI/1.5);
    posori_task->_otg->setMaxAngularAcceleration(3*M_PI);
    posori_task->_otg->setMaxAngularJerk(15*M_PI);

    posori_task->_kp_pos = 100.0;
    posori_task->_kv_pos = 17.0;

    posori_task->_kp_ori = 200.0;
    posori_task->_kv_ori = 23.0;

    // TODO: tune PID gains for force control
    posori_task->_kp_force = 1.0;
    posori_task->_kv_force = 10.0;
    posori_task->_ki_force = 0.7;

    posori_task->_kp_moment = 6.0;
    posori_task->_kv_moment = 8.0;
    posori_task->_ki_moment = 1.0;

    return posori_task;
}
