// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"
#include "../src/Logger.h"
#include "perception/ForceSpaceParticleFilter.h"

#include <iostream>
#include <string>
#include <random>
#include <queue>

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
string JOINT_ANGLES_KEY = "sai2::HapticApplications::03::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::HapticApplications::03::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::HapticApplications::03::simviz::actuators::tau_cmd";

string ROBOT_SENSED_FORCE_KEY = "sai2::HapticApplications::03::simviz::sensors::sensed_force";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

// posori task
// state information
string ROBOT_EE_POS_KEY = "sai2::HapticApplications::03::robot::ee_pos";
string ROBOT_EE_ORI_KEY = "sai2::HapticApplications::03::robot::ee_ori";
string ROBOT_EE_FORCE_KEY = "sai2::HapticApplications::03::robot::ee_force";
string ROBOT_EE_MOMENT_KEY = "sai2::HapticApplications::03::robot::ee_moment";
// user parameters
string EE_POS_IN_LINK_KEY = "sai2::HapticApplications::03::posori::ee_pos_in_link";
string EE_ROT_IN_LINK_KEY = "sai2::HapticApplications::03::posori::ee_rot_in_link";
string ANGULAR_MOTION_AXIS_KEY = "sai2::HapticApplications::03::posori::angular_motion_axis";
string POSORI_WRITE_PARAMS_KEY = "sai2::HapticApplications::03::posori::write_params"; // set to 1 to write parameter changes

// dual proxy
string ROBOT_PROXY_KEY = "sai2::HapticApplications::03::dual_proxy::robot_proxy";
string ROBOT_PROXY_ROT_KEY = "sai2::HapticApplications::03::dual_proxy::robot_proxy_rot";
string HAPTIC_PROXY_KEY = "sai2::HapticApplications::03::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::HapticApplications::03::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::HapticApplications::03::dual_proxy::sigma_force";
string ROBOT_DEFAULT_ROT_KEY = "sai2::HapticApplications::03::dual_proxy::robot_default_rot";
string ROBOT_DEFAULT_POS_KEY = "sai2::HapticApplications::03::dual_proxy::robot_default_pos";

string HAPTIC_DEVICE_READY_KEY = "sai2::HapticApplications::03::dual_proxy::haptic_device_ready";
string CONTROLLER_RUNNING_KEY = "sai2::HapticApplications::03::dual_proxy::controller_running";


RedisClient redis_client;

// particle filter parameters
const int n_particles = 1000;
MatrixXd particle_positions_to_redis = MatrixXd::Zero(3, n_particles);
int force_space_dimension = 0;
int prev_force_space_dimension = 0;
Matrix3d sigma_force = Matrix3d::Zero();
Matrix3d sigma_motion = Matrix3d::Identity();

Vector3d motion_control_pfilter;
Vector3d force_control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

queue<Vector3d> pfilter_motion_control_buffer;
queue<Vector3d> pfilter_force_control_buffer;
queue<Vector3d> pfilter_sensed_force_buffer;
queue<Vector3d> pfilter_sensed_velocity_buffer;

const double control_loop_freq = 1000.0;
const double pfilter_freq = 50.0;
const double freq_ratio_filter_control = pfilter_freq / control_loop_freq;

// set sensor frame transform in end-effector frame
Affine3d sensor_transform_in_link = Affine3d::Identity();
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.034); // TODO: get measurement from borns

// helper function for creating + initializing posori task
unique_ptr<Sai2Primitives::PosOriTask> init_posori(Sai2Model::Sai2Model* robot,
                                                    const std::string link_name,
                                                    const Eigen::Vector3d pos_in_link,
                                                    const Eigen::Matrix3d rot_in_link);

// particle filter loop
void particle_filter();

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {

    if(!flag_simulation) {
        ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::actuators::fgc";
        JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
        JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
        MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
        CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
        ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::force_torque";
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

    // VectorXd q_init(dof);
    // q_init << 0, -30, 0, -130, 0, 100, 0;
    // q_init *= M_PI/180.0;
    // q_init << 0.943667,-0.76595,-2.18203,-1.72349,-0.701489,2.13662,-0.0375883;
    joint_task->_desired_position = robot->_q; // use current robot config as init config

    // posori task
    // set control link and point for posori task
    const string link_name = "end_effector";
//    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.153); // TODO: get measurement from borns
    Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.1); // TODO: get measurement from borns
    Matrix3d rot_in_link = Matrix3d::Identity();
    unique_ptr<Sai2Primitives::PosOriTask> posori_task = init_posori(robot, link_name, pos_in_link, rot_in_link);

    VectorXd posori_task_torques = VectorXd::Zero(dof);

    // flag for setting new posori parameters
    int posori_write_params = 0;

    // track robot's autonomous ori change during surface-surface alignment
    Matrix3d R_proxy_robot = Matrix3d::Identity();

    // normal axis for surface-surface alignment
    Vector3d angular_motion_axis = Vector3d(0.0, 0.0, 1.0);

    // force sensing
    Matrix3d R_link_sensor = Matrix3d::Identity();
    sensor_transform_in_link.translation() = sensor_pos_in_link;
    sensor_transform_in_link.linear() = R_link_sensor;
    posori_task->setForceSensorFrame(link_name, sensor_transform_in_link);

    VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
    VectorXd sensed_force_moment_world_frame = VectorXd::Zero(6);
    VectorXd force_bias = VectorXd::Zero(6);
    double tool_mass = 0;
    Vector3d tool_com = Vector3d::Zero();

    Vector3d init_force = Vector3d::Zero();
    bool first_loop = true;

    if(!flag_simulation) {
        force_bias << -0.0448175, 1.88045, -1.76062, -0.0733607, 0.392116, 0.0397938;
        tool_mass = 0.366843;
        tool_com = Vector3d(0.000607626, -0.00030761, 0.0402244);
    }

    // remove inertial forces from tool
    Vector3d tool_velocity = Vector3d::Zero();
    Vector3d prev_tool_velocity = Vector3d::Zero();
    Vector3d tool_acceleration = Vector3d::Zero();
    Vector3d tool_inertial_forces = Vector3d::Zero();

    // dual proxy parameters and variables
    double k_vir = 250.0;
    double max_force_diff = 0.1;
    double max_force = 10.0;

    int haptic_ready = 0;

    // initialize redis values
    redis_client.set(HAPTIC_DEVICE_READY_KEY, std::to_string(haptic_ready));
    redis_client.setEigenMatrixJSON(EE_POS_IN_LINK_KEY, pos_in_link);
    redis_client.setEigenMatrixJSON(EE_ROT_IN_LINK_KEY, rot_in_link);
    redis_client.setEigenMatrixJSON(ANGULAR_MOTION_AXIS_KEY, angular_motion_axis);
    redis_client.set(POSORI_WRITE_PARAMS_KEY, std::to_string(posori_write_params));

    Vector3d robot_proxy = Vector3d::Zero();
    Matrix3d robot_proxy_rot = Matrix3d::Identity();
    Vector3d haptic_proxy = Vector3d::Zero();
    Vector3d prev_desired_force = Vector3d::Zero();

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

    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment_local_frame);

    // Objects to write to redis
    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

    redis_client.addEigenToWriteCallback(0, HAPTIC_PROXY_KEY, haptic_proxy);
    redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
    redis_client.addIntToWriteCallback(0, FORCE_SPACE_DIMENSION_KEY, force_space_dimension);

    // write internal controller state to redis
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_POS_KEY, posori_task->_current_position);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_ORI_KEY, posori_task->_current_orientation);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_FORCE_KEY, posori_task->_sensed_force);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_MOMENT_KEY, posori_task->_sensed_moment);

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
    VectorXd log_sensed_force_moments = VectorXd::Zero(6);
    Vector3d log_desired_force = Vector3d::Zero();

    Matrix3d log_ee_orientation = posori_task->_current_orientation; // TODO: add to logger

    Vector3d log_posori_sensed_force = Vector3d::Zero();
    Vector3d log_posori_sensed_moment = Vector3d::Zero();

    logger->addVectorToLog(&log_ee_position, "robot_ee_position");
    logger->addVectorToLog(&log_ee_velocity, "robot_ee_velocity");
    logger->addVectorToLog(&log_robot_proxy_position, "robot_proxy_position");
    logger->addVectorToLog(&log_joint_angles, "joint_angles");
    logger->addVectorToLog(&log_joint_velocities, "joint_velocities");
    logger->addVectorToLog(&log_joint_command_torques, "joint_command_torques");
    logger->addVectorToLog(&log_sensed_force_moments, "sensed_forces_moments");
    logger->addVectorToLog(&log_desired_force, "desired_force");

    logger->addVectorToLog(&log_posori_sensed_force, "posori_sensed_force");
    logger->addVectorToLog(&log_posori_sensed_moment, "posori_sensed_moment");

    logger->start();

    // start particle filter thread
    runloop = true;
    redis_client.set(CONTROLLER_RUNNING_KEY,"1");
    thread particle_filter_thread(particle_filter);


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

        // add bias and ee weight to sensed forces
        sensed_force_moment_local_frame -= force_bias;
        Matrix3d R_world_sensor;
        robot->rotation(R_world_sensor, link_name);
        R_world_sensor = R_world_sensor * R_link_sensor;
        Vector3d p_tool_local_frame = tool_mass * R_world_sensor.transpose() * Vector3d(0,0,-9.81);
        sensed_force_moment_local_frame.head(3) += p_tool_local_frame;
        sensed_force_moment_local_frame.tail(3) += tool_com.cross(p_tool_local_frame);

        if(first_loop) {
            init_force = sensed_force_moment_local_frame.head(3);
            first_loop = false;
        }
        sensed_force_moment_local_frame.head(3) -= init_force;

        // update forces for posori task
        posori_task->updateSensedForceAndMoment(sensed_force_moment_local_frame.head(3), sensed_force_moment_local_frame.tail(3));
        sensed_force_moment_world_frame.head(3) = R_world_sensor * sensed_force_moment_local_frame.head(3);
        sensed_force_moment_world_frame.tail(3) = R_world_sensor * sensed_force_moment_local_frame.tail(3);


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

            // surface-surface alignment
            // do not update desired orientation when in contact
            if(force_space_dimension >= 1) {
                Vector3d local_angular_motion_axis = robot_orientation * angular_motion_axis;

                if(controller_counter%500==0)
                {
                    std::cout << "ang motion axis: " << angular_motion_axis << std::endl;
                    std::cout << "robot ori: " << robot_orientation << std::endl;
                    std::cout << "local ang motion axis: " << local_angular_motion_axis << std::endl;
                }

                if(prev_force_space_dimension == 0) {
                    // avoid step response when transitioning from free space to contact (reinitialize OTG)
                    posori_task->_otg->reInitialize(robot_position, robot_orientation);
                    posori_task->setAngularMotionAxis(local_angular_motion_axis);
                }
                else {
                    posori_task->updateAngularMotionAxis(local_angular_motion_axis);
                }
            }
            else {
                if (prev_force_space_dimension >= 1) {
                    // change to full motion control on transition to free space
                    posori_task->setFullAngularMotionControl();
                    // avoid step response when transitioning from contact to free space (reinitialize OTG)
                    posori_task->_otg->reInitialize(robot_position, robot_orientation);
                    // keep track of rotation from robot to proxy frame (not necessarily aligned during local autonomy)
                    R_proxy_robot = robot_proxy_rot.transpose() * robot_orientation;
                }
            }

            // motion
            Vector3d motion_proxy = robot_position + sigma_motion * (robot_proxy - robot_position);
            Matrix3d rotation_proxy = robot_proxy_rot * R_proxy_robot;

            // force
            Vector3d desired_force = k_vir * sigma_force * (robot_proxy - robot_position);
            Vector3d desired_force_diff = desired_force - prev_desired_force;
            if(desired_force_diff.norm() > max_force_diff) {
                desired_force = prev_desired_force + desired_force_diff*max_force_diff/desired_force_diff.norm();
            }
            if(desired_force.norm() > max_force) {
                desired_force *= max_force/desired_force.norm();
            }

            // control
            posori_task->_desired_position = motion_proxy;
//            posori_task->_desired_orientation = robot_proxy_rot;
            posori_task->_desired_orientation = rotation_proxy;
            posori_task->_desired_force = desired_force;

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
            prev_desired_force = desired_force;
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


        // particle filter
        pfilter_motion_control_buffer.push(sigma_motion * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
        pfilter_force_control_buffer.push(sigma_force * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
        // pfilter_motion_control_buffer.push(sigma_motion * posori_task->_Lambda_modified.block<3,3>(0,0) * posori_task->_linear_motion_control * freq_ratio_filter_control);
        // pfilter_force_control_buffer.push(sigma_force * posori_task->_linear_force_control * freq_ratio_filter_control);

        pfilter_sensed_velocity_buffer.push(posori_task->_current_velocity * freq_ratio_filter_control);
        pfilter_sensed_force_buffer.push(sensed_force_moment_world_frame.head(3) * freq_ratio_filter_control);

        motion_control_pfilter += pfilter_motion_control_buffer.back();
        force_control_pfilter += pfilter_force_control_buffer.back();
        measured_velocity_pfilter += pfilter_sensed_velocity_buffer.back();
        measured_force_pfilter += pfilter_sensed_force_buffer.back();

        if(pfilter_motion_control_buffer.size() > 1/freq_ratio_filter_control) {
            motion_control_pfilter -= pfilter_motion_control_buffer.front();
            force_control_pfilter -= pfilter_force_control_buffer.front();
            measured_velocity_pfilter -= pfilter_sensed_velocity_buffer.front();
            measured_force_pfilter -= pfilter_sensed_force_buffer.front();

            pfilter_motion_control_buffer.pop();
            pfilter_force_control_buffer.pop();
            pfilter_sensed_velocity_buffer.pop();
            pfilter_sensed_force_buffer.pop();
        }

        // update logger values
        Vector3d ee_vel = Vector3d::Zero();
        robot->linearVelocity(ee_vel, link_name, pos_in_link);

        log_ee_position = haptic_proxy;
        log_ee_velocity = ee_vel;
        log_robot_proxy_position = robot_proxy;
        log_joint_angles = robot->_q;
        log_joint_velocities = robot->_dq;
        log_joint_command_torques = command_torques;
        log_sensed_force_moments = sensed_force_moment_world_frame;
        log_desired_force = posori_task->_desired_force;

        log_posori_sensed_force = posori_task->_sensed_force;
        log_posori_sensed_moment = posori_task->_sensed_moment;

        controller_counter++;
    }

    // wait for particle filter thread
    particle_filter_thread.join();

    // stop logger
    logger->stop();

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



void particle_filter() {
    // start redis client for particles
    auto redis_client_particles = RedisClient();
    redis_client_particles.connect();

    unsigned long long pf_counter = 0;

    // create particle filter
    auto pfilter = new Sai2Primitives::ForceSpaceParticleFilter(n_particles);

    pfilter->_mean_scatter = 0.0;
    pfilter->_std_scatter = 0.025;

    pfilter->_alpha_add = 0.3;
    pfilter->_alpha_remove = 0.05;

    pfilter->_F_low = 2.0;
    pfilter->_F_high = 6.0;
    pfilter->_v_low = 0.03;
    pfilter->_v_high = 0.07;

    pfilter->_F_low_add = 5.0;
    pfilter->_F_high_add = 10.0;
    pfilter->_v_low_add = 0.02;
    pfilter->_v_high_add = 0.1;

    Vector3d evals = Vector3d::Zero();
    Matrix3d evecs = Matrix3d::Identity();

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
    double current_time = 0;
    double prev_time = 0;
    // double dt = 0;
    bool fTimerDidSleep = true;
    double start_time = timer.elapsedTime(); //secs

    while(runloop) {
        timer.waitForNextLoop();

        pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
        sigma_force = pfilter->getSigmaForce();
        sigma_motion = Matrix3d::Identity() - sigma_force;
        force_space_dimension = pfilter->_force_space_dimension;

        // for(int i=0 ; i<n_particles ; i++)
        // {
        //  particle_positions_to_redis.col(i) = pfilter->_particles[i];
        // }
        // redis_client_particles.setEigenMatrixJSON(PARTICLE_POSITIONS_KEY, particle_positions_to_redis);

        pf_counter++;
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
    std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
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
