// This example tests the haptic device driver and the open-loop bilateral teleoperation controller.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"

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
string JOINT_ANGLES_KEY = "sai2::HapticApplications::01-panda::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::HapticApplications::01-panda::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::HapticApplications::01-panda::simviz::actuators::tau_cmd";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

// dual proxy
string ROBOT_PROXY_KEY = "sai2::HapticApplications::01::dual_proxy::robot_proxy";
string HAPTIC_PROXY_KEY = "sai2::HapticApplications::01::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::HapticApplications::01::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::HapticApplications::01::dual_proxy::sigma_force";

string HAPTIC_DEVICE_READY_KEY = "sai2::HapticApplications::01::dual_proxy::haptic_device_ready";
string CONTROLLER_RUNNING_KEY = "sai2::HapticApplications::01::dual_proxy::controller_running";

int force_space_dimension = 0;
Matrix3d sigma_force = Matrix3d::Zero();

RedisClient redis_client;

// const bool flag_simulation = false;
const bool flag_simulation = true;

int main() {

	if(!flag_simulation)
	{
		ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
	}

	// start redis client local
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	Affine3d T_workd_robot = Affine3d::Identity();
	T_workd_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_workd_robot);

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

	VectorXd q_init(dof);
	q_init << 0, -30, 0, -130, 0, 100, 0;
	q_init *= M_PI/180.0;
	joint_task->_desired_position = q_init;

	// posori task
	const string link_name = "end_effector";
	const Vector3d pos_in_link = Vector3d(0,0,0);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	Vector3d x_init = posori_task->_current_position;
	redis_client.setEigenMatrixJSON(ROBOT_PROXY_KEY, x_init);
	redis_client.setEigenMatrixJSON(HAPTIC_PROXY_KEY, x_init);

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_use_interpolation_flag = true;

	posori_task->_otg->setMaxLinearVelocity(0.30);
	posori_task->_otg->setMaxLinearAcceleration(10.0);
	posori_task->_otg->setMaxLinearJerk(50.0);

	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 23.0;

	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 35.0;

	// dual proxy parameters and variables
	double k_vir = 500.0;
	double max_force_diff = 0.05;
	double max_force = 15.0;

	int haptic_ready = 0;
	redis_client.set(HAPTIC_DEVICE_READY_KEY, "0");

	Vector3d robot_proxy = Vector3d::Zero();
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
	if(!flag_simulation)
	{
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot);
	}

	redis_client.addEigenToReadCallback(0, ROBOT_PROXY_KEY, robot_proxy);
	redis_client.addIntToReadCallback(0, HAPTIC_DEVICE_READY_KEY, haptic_ready);

	// Objects to write to redis
	redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

	redis_client.addEigenToWriteCallback(0, HAPTIC_PROXY_KEY, haptic_proxy);
	redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
	redis_client.addIntToWriteCallback(0, FORCE_SPACE_DIMENSION_KEY, force_space_dimension);

	// start communication thread
	runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");

	// create a timer
	double control_loop_freq = 1000.0;
	unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_loop_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (runloop)
	{
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
		if(flag_simulation)
		{
			robot->updateModel();
			robot->coriolisForce(coriolis);
		}
		else
		{
			robot->updateKinematics();
			robot->_M = mass_from_robot;
			robot->updateInverseInertia();
			coriolis = coriolis_from_robot;
		}

		N_prec.setIdentity(dof,dof);

		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;

		joint_task->updateTaskModel(N_prec);

		if(state == INIT)
		{
			joint_task->updateTaskModel(MatrixXd::Identity(dof,dof));

			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis;

			if(haptic_ready && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.2)
			{
				// Reinitialize controllers
				posori_task->reInitializeTask();
				joint_task->reInitializeTask();

				state = CONTROL;
			}
		}

		else if(state == CONTROL)
		{
			// dual proxy
			Matrix3d sigma_motion = Matrix3d::Identity() - sigma_force;
			posori_task->_sigma_force = sigma_force;
			posori_task->_sigma_position = sigma_motion;

			Vector3d robot_position = posori_task->_current_position;

			Vector3d motion_proxy = robot_position + sigma_motion * (robot_proxy - robot_position);

			Vector3d desired_force = k_vir * sigma_force * (robot_proxy - robot_position);
			Vector3d desired_force_diff = desired_force - prev_desired_force;
			if(desired_force_diff.norm() > max_force_diff)
			{
				desired_force = prev_desired_force + desired_force_diff*max_force_diff/desired_force_diff.norm();
			}
			if(desired_force.norm() > max_force)
			{
				desired_force *= max_force/desired_force.norm();
			}

			// control
			posori_task->_desired_position = motion_proxy;
			posori_task->_desired_force = desired_force;

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// remember values
			prev_desired_force = desired_force;
		}

		// write control torques and dual proxy variables
		robot->position(haptic_proxy, link_name, pos_in_link);
		redis_client.executeWriteCallback(0);

		controller_counter++;
	}

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

