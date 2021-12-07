// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"
#include "../src/Logger.h"
#include "perception/ForceSpaceParticleFilter.h"
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <random>
#include <queue>

#define NUM_RIGID_BODIES        2
#define RIGID_BODY_OF_INTEREST  0

#define LAST_JOINT_MAX_ROT       120.0 * (M_PI / 180.0) // max angle of last joint on panda (in radians)
#define LAST_JOINT_MIN_ROT      -120.0 * (M_PI / 180.0) // min angle of last joint on panda (in radians)

// control modes
enum {
    INIT,
    AUTO_CONTROL,
    HAPTIC_CONTROL
};

// primitivies
enum {
    GO_TO_POINT, 
    MAKE_CONTACT,
    ALIGN,
    ENGAGE,
    SCREW,
    TIGHTEN
};

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";

// redis keys:
// robot local control loop
string JOINT_ANGLES_KEY = "sai2::LearningSkills::lead_control::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::LearningSkills::lead_control::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::LearningSkills::lead_control::simviz::actuators::tau_cmd";
string ROBOT_SENSED_FORCE_KEY = "sai2::LearningSkills::lead_control::simviz::sensors::sensed_force";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

// posori task
// read perception state
string POS_RIGID_BODIES_KEY = "sai2::optitrack::pos_rigid_bodies";
string ORI_RIGID_BODIES_KEY = "sai2::optitrack::ori_rigid_bodies";
// write state information
string ROBOT_EE_POS_KEY = "sai2::LearningSkills::lead_control::robot::ee_pos";
string ROBOT_EE_ORI_KEY = "sai2::LearningSkills::lead_control::robot::ee_ori";
string ROBOT_EE_FORCE_KEY = "sai2::LearningSkills::lead_control::robot::ee_force";
string ROBOT_EE_MOMENT_KEY = "sai2::LearningSkills::lead_control::robot::ee_moment";

// dual proxy
string ROBOT_PROXY_KEY = "sai2::LearningSkills::lead_control::dual_proxy::robot_proxy";
string ROBOT_PROXY_ROT_KEY = "sai2::LearningSkills::lead_control::dual_proxy::robot_proxy_rot";
string HAPTIC_PROXY_KEY = "sai2::LearningSkills::lead_control::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::LearningSkills::lead_control::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::LearningSkills::lead_control::dual_proxy::sigma_force";
string ROBOT_DEFAULT_ROT_KEY = "sai2::LearningSkills::lead_control::dual_proxy::robot_default_rot";
string ROBOT_DEFAULT_POS_KEY = "sai2::LearningSkills::lead_control::dual_proxy::robot_default_pos";

string HAPTIC_DEVICE_READY_KEY = "sai2::LearningSkills::lead_control::dual_proxy::haptic_device_ready";
string CONTROLLER_RUNNING_KEY = "sai2::LearningSkills::lead_control::dual_proxy::controller_running"; // 0 is off, 1 is auto control, 2 is haptic control

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
const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0333);

// helper function for creating + initializing posori task
unique_ptr<Sai2Primitives::PosOriTask> init_posori(Sai2Model::Sai2Model* robot,
                                                    const std::string link_name,
                                                    const Eigen::Vector3d pos_in_link,
                                                    const Eigen::Matrix3d rot_in_link);

// particle filter loop
void particle_filter();

const bool flag_simulation = false;
// const bool flag_simulation = true;

int main(int argc, char ** argv) {

    std::string robot_name;
    std::string object_name;

    if(!flag_simulation) {
        
        if(argc < 3)
        { 
            fprintf( stderr, ">>> Usage: %s [ROBOT_NAME] [OBJECT_NAME]\n\n", argv[0] );
            fprintf( stderr, "Robot name options: \n    > Bonnie\n    > Clyde\n\n");
            fprintf( stderr, "Object options:\n    > bottle\n    > cap\n    > bulb\n\n");
            return 0;
        }

        robot_name = argv[1];
        object_name = argv[2];

        if(robot_name == "Clyde")
        {
            ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
            JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
            JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
            MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
            CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
            ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Clyde::force_torque";  
        }
        else if(robot_name == "Bonnie")
        {
            ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
            JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
            JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
            MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
            CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
            ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
        }
    }

    // start redis client local
    redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    int state = INIT;
    int primitive = GO_TO_POINT;
    // int primitive = SCREW;

    // load robots
    Affine3d T_world_robot = Affine3d::Identity();
    T_world_robot.translation() = Vector3d(0, 0, 0);
    auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);
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
    // q_init << 0, -30, 0, -130, 0, 100, 0;
    // q_init *= M_PI/180.0;
    q_init = robot->_q;
    joint_task->_desired_position = q_init; // use current robot config as init config

    // posori task
    // set control link and point for posori task
    const string link_name = "end_effector";
    // Vector3d pos_in_link = Vector3d(0.0, 0.015, 0.28);
    // Vector3d pos_in_link = Vector3d(0.0, 0.0, -0.15); // optitrack calibration
    Vector3d pos_in_link = Vector3d::Zero();
    if(robot_name == "Clyde") pos_in_link = Vector3d(0.16151, -0.0294118, 0.137382); // cap grasp
    if(robot_name == "Bonnie") pos_in_link = Vector3d(0.131569, -0.00721577, 0.161644); // cap grasp
    Matrix3d rot_in_link = Matrix3d::Identity();
    unique_ptr<Sai2Primitives::PosOriTask> posori_task = init_posori(robot, link_name, pos_in_link, rot_in_link);

    Matrix3d R_init = posori_task->_current_orientation;

    VectorXd posori_task_torques = VectorXd::Zero(dof);

    // track robot's autonomous ori change during surface-surface alignment
    Matrix3d R_proxy_robot = Matrix3d::Identity();

    // primitive parameters
    unsigned long long make_contact_counter = 0;
    unsigned long long screw_counter = 0;
    // check initial screw direction
    bool screw_back_off = false; // false to screw forward, true to back off
    double last_joint_angle_init = q_init(dof-1);
    if(last_joint_angle_init >= 0.0) {
        screw_back_off = false;
    }
    else {
        screw_back_off = true;
    }

    // initialize desired robot pose as current pose
    Vector3d x_des = posori_task->_current_position;
    Matrix3d ori_des = posori_task->_current_orientation;

    // rigid body positions from perception
    double optitrack_angle = 0.0;
    Vector3d pos_robot_ref_in_optitrack_frame = Vector3d::Zero();
    Vector3d pos_robot_ref_in_robot_frame = Vector3d::Zero();

    if(robot_name == "Clyde")
    {
        // angle from robot base frame +X to optitrack -Z w/ positive sense about robot base frame +Z
        optitrack_angle = -17.5;

        // measure position of reference point on robot in both optitrack and robot frames
        // the robot ref y-position in optitrack frame should match z-position in robot frame
        pos_robot_ref_in_optitrack_frame = Vector3d(0.389816, 0.784016, 0.506797);
        pos_robot_ref_in_robot_frame = Vector3d(0.254116, 0.117725, 0.786114);

        // 127.0.0.1:6379> get sai2::optitrack::pos_rigid_bodies
        // "[[0.183147,0.591577,0.334187],[0.389816,0.784016,0.506797]]" // cap and robot ref in optitrack frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_pos
        // "[0.254116,0.117725,0.786114]" // robot ref in robot frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_ori
        // "[[0.448861,0.658382,0.604200],[0.834585,-0.550511,-0.020136],[0.319361,0.513294,-0.796578]]"
    }

    else if(robot_name == "Bonnie")
    {
        // angle from robot base frame +X to optitrack -Z w/ positive sense about robot base frame +Z
        optitrack_angle = 62.5;

        // pos_robot_ref_in_optitrack_frame = Vector3d(-0.508397, 0.847871, 0.313138);
        // pos_robot_ref_in_robot_frame = Vector3d(0.048436, 0.275109, 0.922579);

        // measure position of reference point on robot in both optitrack and robot frames
        // the robot ref y-position in optitrack frame should match z-position in robot frame
        pos_robot_ref_in_optitrack_frame = Vector3d(-0.140498, 0.883456, 0.233513);
        pos_robot_ref_in_robot_frame = Vector3d(0.385345, 0.216355, 0.884632);

        // 127.0.0.1:6379> get sai2::optitrack::pos_rigid_bodies
        // "[[-0.015708,0.571941,0.190267],[-0.140498,0.883456,0.233513]]" // cap and robot ref in optitrack frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_pos
        // "[0.385345,0.216355,0.884632]" // robot ref in robot frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_ori
        // "[[0.980947,0.194037,0.009617],[0.192927,-0.967129,-0.165650],[-0.022841,0.164349,-0.986138]]"
    }

    // set offset from robot frame to rigid body perception (optitrack) frame
    Matrix3d rot_optitrack_in_robot_frame;
    rot_optitrack_in_robot_frame <<  sin(optitrack_angle * M_PI / 180.0),    0,  -cos(optitrack_angle * M_PI / 180.0),
                                     -cos(optitrack_angle * M_PI / 180.0),   0,  -sin(optitrack_angle * M_PI / 180.0),
                                     0,                                      1,   0;
    // list of rigid body poses in world
    // TODO: fix crash from not adding extra row in reading optitrack positions
    MatrixXd pos_rigid_bodies(NUM_RIGID_BODIES+1, 3);
    MatrixXd ori_rigid_bodies(NUM_RIGID_BODIES+1, 4);
    pos_rigid_bodies.setZero();
    ori_rigid_bodies.setZero();

    // rigid body pose of interest
    Vector3d        pos_rigid_body_in_optitrack_frame          = Vector3d::Zero();
    Matrix3d        ori_rigid_body_in_optitrack_frame          = Matrix3d::Identity();
    Quaterniond     ori_rigid_body_in_optitrack_frame_quat     = Quaterniond::Identity();
    Vector3d        pos_rigid_body_in_robot_frame              = Vector3d::Zero();
    Matrix3d        ori_rigid_body_in_robot_frame              = Matrix3d::Identity();


    // measure cap position in end-effector frame (for pos_in_link in posori task)
    Matrix3d rot_robot_ref_in_robot_frame = Matrix3d::Identity();
    Vector3d pos_cap_in_optitrack_frame = Vector3d::Zero();
    Vector3d rel_pos_cap_in_robot_frame = Vector3d::Zero();

    if(robot_name == "Clyde") {

        rot_robot_ref_in_robot_frame << 0.448861,  0.658382,  0.604200,
                                        0.834585, -0.550511, -0.020136,
                                        0.319361,  0.513294, -0.796578;
        pos_cap_in_optitrack_frame = Vector3d(0.183147, 0.591577, 0.334187);
        
        // 127.0.0.1:6379> get sai2::optitrack::pos_rigid_bodies
        // "[[0.183147,0.591577,0.334187],[0.389816,0.784016,0.506797]]" // cap and robot ref in optitrack frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_pos
        // "[0.254116,0.117725,0.786114]" // robot ref in robot frame
        // 127.0.0.1:6379> get sai2::LearningSkills::lead_control::robot::ee_ori
        // "[[0.448861,0.658382,0.604200],[0.834585,-0.550511,-0.020136],[0.319361,0.513294,-0.796578]]"

    }
    if(robot_name == "Bonnie") {
        rot_robot_ref_in_robot_frame <<  0.980947,  0.194037,  0.009617,
                                         0.192927, -0.967129, -0.165650,
                                        -0.022841,  0.164349, -0.986138;
        pos_cap_in_optitrack_frame = Vector3d(-0.015708, 0.571941, 0.190267);
    }

    Vector3d rel_pos_robot_ref_in_ee_frame = Vector3d(0.0, 0.0, -0.15);
    rel_pos_cap_in_robot_frame = rot_optitrack_in_robot_frame * (pos_cap_in_optitrack_frame - pos_robot_ref_in_optitrack_frame) - (rot_robot_ref_in_robot_frame * -rel_pos_robot_ref_in_ee_frame);
    
    Vector3d rel_pos_cap_in_ee_frame = rot_robot_ref_in_robot_frame.transpose() * rel_pos_cap_in_robot_frame;

    // rotation from cap to robot ee frame (depends on Bonnie or Clyde)
    Matrix3d rot_rigid_body_in_ee_frame = Matrix3d::Identity();
    if(robot_name == "Clyde") {
        rot_rigid_body_in_ee_frame <<    0.0, 1.0,  0.0,
                                         0.0, 0.0, -1.0,
                                        -1.0, 0.0,  0.0;
    }
    if(robot_name == "Bonnie") {
        rot_rigid_body_in_ee_frame <<    0.0, -1.0,  0.0,
                                         0.0,  0.0, -1.0,
                                         1.0,  0.0,  0.0;
    }

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
        if (object_name == "bottle"){
            tool_com = Vector3d(0.104441, -0.00357995, 0.0451504);        
            tool_mass = 1.38862;
            force_bias << -2.38828, 3.18213, 1.63922, -0.0221789, 0.2543, 0.0397122;
        }
        else if (object_name == "cap"){
            tool_com = Vector3d(0.111174, -0.00247079, 0.037597);
            tool_mass = 1.30151;
            force_bias << 0.164016, 1.99319, -1.08685, -0.0526554, 0.322687, 0.0513417;
        }
        else if (object_name == "bulb"){
            tool_com = Vector3d(0.111174, -0.00247079, 0.037597);
            tool_mass = 1.30151;
            force_bias << 0.164016, 1.99319, -1.08685, -0.0526554, 0.322687, 0.0513417;
            fprintf(stderr, "\n WARNING: Haven't calibrated the sensor for bulb on Clyde yet \nUsing cap parameters for now!\n\n");
        }
        else{
            fprintf(stderr, "\n\n>>> Hey!! I think you need to calibrate the FT sensor for this new object\n\n");
        }
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

    int haptic_device_ready = 0;
    redis_client.set(HAPTIC_DEVICE_READY_KEY, "0");

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
    redis_client.addIntToReadCallback(0, HAPTIC_DEVICE_READY_KEY, haptic_device_ready);

    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment_local_frame);

    redis_client.addEigenToReadCallback(0, POS_RIGID_BODIES_KEY, pos_rigid_bodies);
    redis_client.addEigenToReadCallback(0, ORI_RIGID_BODIES_KEY, ori_rigid_bodies);

    // Objects to write to redis
    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

    redis_client.addEigenToWriteCallback(0, HAPTIC_PROXY_KEY, haptic_proxy);
    redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
    redis_client.addIntToWriteCallback(0, FORCE_SPACE_DIMENSION_KEY, force_space_dimension);

    // write internal controller state to redis
    Vector3d robot_ee_pos = posori_task->_current_position;
    Matrix3d robot_ee_ori = posori_task->_current_orientation;
    Vector3d robot_ee_force = posori_task->_sensed_force;
    Vector3d robot_ee_moment = posori_task->_sensed_moment;
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_POS_KEY, robot_ee_pos);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_ORI_KEY, robot_ee_ori);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_FORCE_KEY, robot_ee_force);
    redis_client.addEigenToWriteCallback(0, ROBOT_EE_MOMENT_KEY, robot_ee_moment);

    // setup data logging
    string folder = "../../lead_control_robot/data_logging/data/";
    string filename = "data";
    auto logger = new Logging::Logger(100000, folder + filename);
    
    Vector3d log_ee_position = robot_ee_pos;
    Vector3d log_ee_velocity = Vector3d::Zero();
    Vector3d log_robot_proxy_position = robot_proxy;
    VectorXd log_joint_angles = robot->_q;
    VectorXd log_joint_velocities = robot->_dq;
    VectorXd log_joint_command_torques = command_torques;
    VectorXd log_sensed_force_moments = VectorXd::Zero(6);
    Vector3d log_desired_force = Vector3d::Zero();

    Matrix3d log_ee_orientation = robot_ee_ori; // TODO: add to logger

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
    // redis_client.set(CONTROLLER_RUNNING_KEY,"1");
    redis_client.set(CONTROLLER_RUNNING_KEY,"2");
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

        // if(primitive == SCREW) posori_task->removeTaskJacobianColumn(dof-1);

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

        // update rigid body poses from perception
        Vector3d rigid_body_offset_in_optitrack_frame   = Vector3d::Zero();
        // rigid_body_offset_in_optitrack_frame            = Vector3d(0.046, 0.14, 0.046);
        // rigid_body_offset_in_optitrack_frame            = Vector3d(0.06, 0.14, 0.032);
        rigid_body_offset_in_optitrack_frame            = Vector3d(0.04, 0.14, 0.062);

        pos_rigid_body_in_optitrack_frame               = pos_rigid_bodies.row(RIGID_BODY_OF_INTEREST);
        pos_rigid_body_in_optitrack_frame               += rigid_body_offset_in_optitrack_frame;

        ori_rigid_body_in_optitrack_frame_quat.coeffs() = ori_rigid_bodies.row(RIGID_BODY_OF_INTEREST);
        ori_rigid_body_in_optitrack_frame               = ori_rigid_body_in_optitrack_frame_quat.toRotationMatrix(); // convert quaternion to rotation matrix
        // transform rigid body poses to robot frame
        pos_rigid_body_in_robot_frame = rot_optitrack_in_robot_frame * (pos_rigid_body_in_optitrack_frame - pos_robot_ref_in_optitrack_frame) + pos_robot_ref_in_robot_frame;
        ori_rigid_body_in_robot_frame = rot_optitrack_in_robot_frame * ori_rigid_body_in_optitrack_frame;

        if(state == INIT) {
            joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            state = AUTO_CONTROL;

            std::cout << "Entering AUTO control state" << std::endl;

            redis_client.set(CONTROLLER_RUNNING_KEY, "1"); // set to auto control mode

            posori_task->reInitializeTask();
            joint_task->reInitializeTask();

            if(haptic_device_ready && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2) {
                // Reinitialize controllers
                posori_task->reInitializeTask();
                joint_task->reInitializeTask();

                joint_task->_kp = 50.0;
                joint_task->_kv = 13.0;
                joint_task->_ki = 0.0;

                state = HAPTIC_CONTROL;

                std::cout << "Entering HAPTIC control state" << std::endl;

                redis_client.set(CONTROLLER_RUNNING_KEY,"2"); // set to haptic control mode
            }
        }

        else if(state == AUTO_CONTROL) {
            Vector3d robot_position = posori_task->_current_position;
            Matrix3d robot_orientation = posori_task->_current_orientation;
            Vector3d desired_force = Vector3d::Zero();

            Matrix3d desired_ori = robot_orientation;

            // go to the object of interest
            if(primitive == GO_TO_POINT) {
                // update desired robot position as rigid body position
                x_des = pos_rigid_body_in_robot_frame;
                // x_des(2) -= 0.05;

                // align robot ee frame with optitrack object axes
                // ori_des.col(0) = -ori_rigid_body_in_robot_frame.col(2);
                // ori_des.col(1) =  ori_rigid_body_in_robot_frame.col(0);
                // ori_des.col(2) = -ori_rigid_body_in_robot_frame.col(1);

                // ori_des = ori_rigid_body_in_robot_frame * rot_rigid_body_in_ee_frame.transpose();
                ori_des = ori_des * AngleAxisd(20.0 * M_PI / 180.0, Vector3d::UnitX());

                if((robot_position - x_des).norm() < 3e-2) {
                    posori_task->reInitializeTask();
                    // primitive = MAKE_CONTACT;
                    cout << "transitioning from GO_TO_POINT to MAKE_CONTACT" << endl << endl;
                }
            }
            // make contact with the object of interest, assuming contact is in local +Z direction
            else if(primitive == MAKE_CONTACT) {
                // update desired robot position as rigid body position
                x_des = pos_rigid_body_in_robot_frame;

                // move towards contact until force is sensed
                x_des(2) -= (make_contact_counter * 0.005) / control_loop_freq;

                // align robot ee frame with optitrack object axes
                // ori_des.col(0) = -ori_rigid_body_in_robot_frame.col(2);
                // ori_des.col(1) =  ori_rigid_body_in_robot_frame.col(0);
                // ori_des.col(2) = -ori_rigid_body_in_robot_frame.col(1);

                ori_des = ori_rigid_body_in_robot_frame * rot_rigid_body_in_ee_frame.transpose();
                ori_des = ori_des * AngleAxisd(20.0 * M_PI / 180.0, Vector3d::UnitX());

                // Vector3d force_axis = robot_orientation.col(2);
                // posori_task->setForceAxis(force_axis);

                // Vector3d desired_force_local = Vector3d(0.0, 0.0, -3.0);
                // desired_force = robot_orientation * desired_force_local;
                // desired_force = desired_force_local;

                // if(posori_task->_sensed_force(2) < -1.0) {
                if(posori_task->_sensed_force.norm() > 1.0) {
                    posori_task->reInitializeTask();
                    primitive = SCREW;
                    make_contact_counter = 0;
                    cout << "transitioning from MAKE_CONTACT to SCREW" << endl << endl;
                }

                ++make_contact_counter;
            }
            else if(primitive == ALIGN) {
                continue;
            }
            else if(primitive == SCREW) {
                // update desired robot position as rigid body position
                x_des = pos_rigid_body_in_robot_frame;
                x_des(2) += 0.2;

                // align robot ee frame with optitrack object axes
                // ori_des.col(0) = -ori_rigid_body_in_robot_frame.col(2);
                // ori_des.col(1) =  ori_rigid_body_in_robot_frame.col(0);
                // ori_des.col(2) = -ori_rigid_body_in_robot_frame.col(1);

                ori_des = ori_rigid_body_in_robot_frame * rot_rigid_body_in_ee_frame.transpose();
                // ori_des = ori_des * AngleAxisd(20.0 * M_PI / 180.0, Vector3d::UnitX());

                // double last_joint_angle = joint_task->_current_position(dof-1);
                // // double last_joint_angle_to_limit = 0.0;
                // if(screw_back_off == false) {
                //     // screw forward until joint limit
                //     if(last_joint_angle < LAST_JOINT_MAX_ROT) {
                //         // last_joint_angle_to_limit = LAST_JOINT_MAX_ROT - last_joint_angle;
                //         // ori_des = R_init * AngleAxisd(screw_counter * (M_PI / 4.0) / control_loop_freq, Vector3d::UnitZ());
                //         ori_des = ori_des * AngleAxisd(screw_counter * (M_PI / 8.0) / control_loop_freq, ori_rigid_body_in_robot_frame * Vector3d::UnitY());
                //         if(controller_counter % 1000 == 0) cout << "SCREWING" << endl << last_joint_angle * 180.0 / M_PI << endl << endl;
                //         // joint_task->_desired_position(dof-1) += (M_PI / 4.0) / control_loop_freq;
                //     }
                //     else {
                //         screw_back_off = true;
                //         screw_counter = 0;
                //         R_init = robot_orientation;
                //     }
                // }
                // // backing off
                // else {
                //     // back off until joint limit
                //     if(last_joint_angle > LAST_JOINT_MIN_ROT) {
                //         // last_joint_angle_to_limit = LAST_JOINT_MIN_ROT - last_joint_angle;
                //         // ori_des = R_init * AngleAxisd(screw_counter * (M_PI / 4.0) / control_loop_freq, -Vector3d::UnitZ());
                //         ori_des = ori_des * AngleAxisd(screw_counter * (M_PI / 8.0) / control_loop_freq, -ori_rigid_body_in_robot_frame * Vector3d::UnitY());
                //         if(controller_counter % 1000 == 0) cout << "BACKING OFF" << endl << endl;
                //         // joint_task->_desired_position(dof-1) = -LAST_JOINT_MAX_ROT;
                //     }
                //     else {
                //         screw_back_off = false;
                //         screw_counter = 0;
                //         R_init = robot_orientation;
                //     }
                // }

                // screw forward
                if(screw_back_off == false) {
                    ori_des = ori_des * AngleAxisd(M_PI / 2.0, ori_rigid_body_in_robot_frame *  Vector3d::UnitY());
                    if(controller_counter % 1000 == 0) cout << "SCREWING" << endl << endl;
                    if(posori_task->goalOrientationReached(3e-2, true)) {
                        screw_back_off = true;
                        // screw_counter = 0;
                        R_init = robot_orientation;
                    }
                }
                // back off
                else {
                    ori_des = ori_des * AngleAxisd(M_PI / 2.0, ori_rigid_body_in_robot_frame * -Vector3d::UnitY());
                    if(controller_counter % 1000 == 0) cout << "SCREWING" << endl << endl;
                    if(posori_task->goalOrientationReached(3e-2, true)) {
                        screw_back_off = false;
                        // screw_counter = 0;
                        R_init = robot_orientation;
                    }
                }

                // desired_ori = robot_orientation * AngleAxisd(last_joint_angle_to_limit, Vector3d::UnitZ());
                // posori_task->_desired_orientation = desired_ori;
                ++screw_counter;
            }

            // dual proxy
            // posori_task->_sigma_force = sigma_force;
            // posori_task->_sigma_position = sigma_motion;

            // Vector3d motion_proxy = robot_position + sigma_motion * (x_des - robot_position);
            // Vector3d motion_proxy = robot_position + posori_task->_sigma_position * (x_des - robot_position);

            // desired_force = k_vir * sigma_force * (x_des - robot_position);
            Vector3d desired_force_diff = desired_force - prev_desired_force;
            if(desired_force_diff.norm() > max_force_diff) {
                desired_force = prev_desired_force + desired_force_diff*max_force_diff/desired_force_diff.norm();
            }
            if(desired_force.norm() > max_force) {
                desired_force *= max_force/desired_force.norm();
            }

            // control
            // posori_task->_desired_position = motion_proxy;
            posori_task->_desired_position = x_des;
            posori_task->_desired_force = desired_force;
            // posori_task->_desired_orientation = robot_proxy_rot; // comment to keep constant orientation
            posori_task->_desired_orientation = ori_des;

            if(controller_counter % 1000 == 0) {
                cout << "pos des = " << endl << x_des << endl << endl;
                cout << "ori des = " << endl << ori_des << endl << endl;
                cout << "des force = " << endl << desired_force << endl << endl;
                cout << "robot pos = " << endl << robot_position << endl << endl;
                cout << "robot pos to x des norm = " << endl << (robot_position - x_des).norm() << endl << endl;
                cout << "primitive = " << endl << primitive << endl << endl;
                cout << "rel_pos_cap_in_ee_frame = " << endl << rel_pos_cap_in_ee_frame << endl << endl;
            }

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

            command_torques.setZero();

            // remember values
            prev_desired_force = desired_force;

          //   // switch to haptic control in failure state
          //   if((x_des - robot_position).norm() < 0.05)
          //   {
          //    state = HAPTIC_CONTROL;

          //    std::cout << "Entering HAPTIC control state" << std::endl;

                // redis_client.set(CONTROLLER_RUNNING_KEY, "2"); // set to haptic control mode
          //   }
        }

        else if(state == HAPTIC_CONTROL) {
            // dual proxy
            posori_task->_sigma_force = sigma_force;
            posori_task->_sigma_position = sigma_motion;

            Vector3d robot_position = posori_task->_current_position;
            Vector3d motion_proxy = robot_position + sigma_motion * (robot_proxy - robot_position);

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
            posori_task->_desired_force = desired_force;
            posori_task->_desired_orientation = robot_proxy_rot;

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

            // switch to haptic control in failure state
            if(haptic_device_ready == 2)
            {
                state = AUTO_CONTROL;

                std::cout << "Entering AUTO control state" << std::endl;

                redis_client.set(CONTROLLER_RUNNING_KEY, "1"); // set to auto control mode
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
        robot_ee_pos = posori_task->_current_position;
        robot_ee_ori = posori_task->_current_orientation;
        robot_ee_force = posori_task->_sensed_force;
        robot_ee_moment = posori_task->_sensed_moment;

        log_ee_position = haptic_proxy;
        log_ee_velocity = ee_vel;
        log_robot_proxy_position = robot_proxy;
        log_joint_angles = robot->_q;
        log_joint_velocities = robot->_dq;
        log_joint_command_torques = command_torques;
        log_sensed_force_moments = sensed_force_moment_world_frame;
        log_desired_force = posori_task->_desired_force;

        log_posori_sensed_force = robot_ee_force;
        log_posori_sensed_moment = robot_ee_moment;
    
        controller_counter++;
    }

    // wait for particle filter thread
    particle_filter_thread.join();

    // stop logger
    logger->stop();

    //// Send zero force/torque to robot ////
    command_torques.setZero();
    redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);
    redis_client.set(CONTROLLER_RUNNING_KEY, "0");

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    delete robot;
    delete joint_task;
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
    pfilter->_v_low = 0.02;
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

    posori_task->_otg->setMaxLinearVelocity(0.075);
    posori_task->_otg->setMaxLinearAcceleration(0.25);
    posori_task->_otg->setMaxLinearJerk(1.25);

    posori_task->_otg->setMaxAngularVelocity(M_PI/6.0);
    posori_task->_otg->setMaxAngularAcceleration(0.75*M_PI);
    posori_task->_otg->setMaxAngularJerk(4.0*M_PI);

    posori_task->_kp_pos = 100.0;
    posori_task->_kv_pos = 17.0;

    posori_task->_kp_ori = 200.0;
    posori_task->_kv_ori = 23.0;

    posori_task->_kp_force = 1.0;
    posori_task->_kv_force = 10.0;
    posori_task->_ki_force = 0.7;

    posori_task->_kp_moment = 6.0;
    posori_task->_kv_moment = 8.0;
    posori_task->_ki_moment = 1.0;

    return posori_task;
}
