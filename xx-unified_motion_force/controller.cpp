#include <iostream>
#include <string>
#include <csignal>
#include <utility>

#include "Sai2Model.h"
#include <dynamics3d.h>

#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"

#include "keys.h"

using namespace Eigen;

////////////////////// CONSTANTS //////////////////////

/** Redis write callback ID: used for setting keys on each controller cycle  */
constexpr int CONTROL_WRITE_CALLBACK_ID = 0;

/** Redis write callback ID: used for setting keys for the first time  */
constexpr int INIT_WRITE_CALLBACK_ID = 1;

/** Redis write callback ID: used for setting keys each time joint task is initialized  */
constexpr int INIT_JOINT_TASK_WRITE_CALLBACK_ID = 2;

/** Redis write callback ID: used for setting keys each time posori task is initialized */
constexpr int INIT_POSORI_TASK_WRITE_CALLBACK_ID = 3;

/** Redis read callback ID: used to grab updated values on each controller cycle */
constexpr int READ_CALLBACK_ID = 0;

/** Flag to determine if we are in simulation or grabbing values from the real robot */
constexpr bool flag_simulation = true;
// constexpr const bool flag_simulation = false;

/** Damping when dragging the robot in the floating task */
constexpr double FLOATING_TASK_KV = 2.5;

// Joint redis keys: select the correct key if we are in simulation or not
constexpr const char *JOINT_ANGLES_KEY = (flag_simulation) ? SIM_JOINT_ANGLES_KEY : HW_JOINT_ANGLES_KEY;
constexpr const char *JOINT_VELOCITIES_KEY = (flag_simulation) ? SIM_JOINT_VELOCITIES_KEY : HW_JOINT_VELOCITIES_KEY;
constexpr const char *JOINT_TORQUES_COMMANDED_KEY = (flag_simulation) ? SIM_JOINT_TORQUES_COMMANDED_KEY : HW_JOINT_TORQUES_COMMANDED_KEY;
// TODO: add !flag_sim redis keys here (robot sensed force)

////////////////// GLOBAL VARIABLES //////////////////
bool runloop = false;
std::string currentPrimitive = PRIMITIVE_JOINT_TASK;
RedisClient redis_client;

////////////////////// FUNCTIONS //////////////////////
/**
 * Custom signal handler: used here to terminate the controller.
 * @param signal The signal (e.g. SIGINT) that was raised.
 */
void sighandler(int)
{
    runloop = false;
}


////////////////// JOINT TASK VARIABLES //////////////////
/** Kp values for each joint in nonisotropic mode */
Eigen::VectorXd joint_kp_nonisotropic;

/** Kv values for each joint in nonisotropic mode */
Eigen::VectorXd joint_kv_nonisotropic;

/** Flag to use OTG interpolation or not */
int joint_use_interpolation;

/** JointTask OTG max velocity */
double joint_interpolation_max_velocity;

/** JointTask OTG max acceleration */
double joint_interpolation_max_acceleration;

/** JointTask OTG max jerk */
double joint_interpolation_max_jerk;

/** Flag to use velocity saturation or not */
int joint_use_velocity_saturation;

/** Flag to use isotropic gains or not */
int joint_use_isotropic_gains;

/** Current dynamic decoupling mode (full, bounded_inertia_estimate, or none) */
std::string joint_dynamic_decoupling_mode;

/**
 * Initializes the JointTask with default gains and settings and sets up the Redis callbacks.
 * @param joint_task    An uninitialized JointTask instance
 * @param redis_client  A RedisClient instance to set up callbacks
 */
void init_joint_task(Sai2Primitives::JointTask *joint_task, RedisClient& redis_client)
{
    int dof = joint_task->_robot->dof();

    // initialize global variables
    joint_kp_nonisotropic = 50.0 * VectorXd::Ones(dof);
    joint_kv_nonisotropic = 13.0 * VectorXd::Ones(dof);
    // joint_use_interpolation = 0;
    joint_use_interpolation = true;
    joint_interpolation_max_velocity = M_PI / 3;
    joint_interpolation_max_acceleration = M_PI;
    joint_interpolation_max_jerk = 3 * M_PI;
    joint_use_velocity_saturation = false;
    joint_use_isotropic_gains = true;
    joint_dynamic_decoupling_mode = "full";

    // initialize joint_task object
    joint_task->_kp = 50.0;
    joint_task->_kv = 13.0;
    joint_task->setNonIsotropicGains(joint_kp_nonisotropic, joint_kv_nonisotropic, VectorXd::Zero(dof));
    joint_task->_desired_position = joint_task->_robot->_q;
    joint_task->_use_isotropic_gains = bool(joint_use_isotropic_gains);
    joint_task->_use_velocity_saturation_flag = bool(joint_use_velocity_saturation);
    joint_task->_use_interpolation_flag = bool(joint_use_interpolation);
    joint_task->setDynamicDecouplingFull();
    joint_task->_otg->setMaxVelocity(joint_interpolation_max_velocity);
    joint_task->_otg->setMaxAcceleration(joint_interpolation_max_acceleration);
    joint_task->_otg->setMaxJerk(joint_interpolation_max_jerk);

    // update values when we read all parameters on a new controller cycle
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, KP_NON_ISOTROPIC_JOINT_KEY, joint_kp_nonisotropic);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, KV_NON_ISOTROPIC_JOINT_KEY, joint_kv_nonisotropic);
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, JOINT_USE_INTERPOLATION, joint_use_interpolation);
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, USE_ISOTROPIC_JOINT_GAINS_KEY, joint_use_isotropic_gains);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, JOINT_INTERPOLATION_MAX_VEL, joint_interpolation_max_velocity);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, JOINT_INTERPOLATION_MAX_ACCEL, joint_interpolation_max_acceleration);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, JOINT_INTERPOLATION_MAX_JERK, joint_interpolation_max_jerk);
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, USE_VEL_SAT_JOINT_KEY, joint_use_velocity_saturation);
    redis_client.addStringToReadCallback(READ_CALLBACK_ID, DYN_DEC_JOINT_KEY, joint_dynamic_decoupling_mode);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_JOINT_POS_KEY, joint_task->_desired_position);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, VEL_SAT_JOINT_KEY, joint_task->_saturation_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_JOINT_KEY, joint_task->_kp);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_JOINT_KEY, joint_task->_kv);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_NON_ISOTROPIC_JOINT_KEY, joint_kp_nonisotropic);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_NON_ISOTROPIC_JOINT_KEY, joint_kv_nonisotropic);
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, JOINT_USE_INTERPOLATION, joint_use_interpolation);
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, USE_ISOTROPIC_JOINT_GAINS_KEY, joint_use_isotropic_gains);
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, USE_VEL_SAT_JOINT_KEY, joint_use_velocity_saturation);
    redis_client.addStringToWriteCallback(INIT_WRITE_CALLBACK_ID, DYN_DEC_JOINT_KEY, joint_dynamic_decoupling_mode);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_JOINT_POS_KEY, joint_task->_desired_position);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, VEL_SAT_JOINT_KEY, joint_task->_saturation_velocity);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, JOINT_INTERPOLATION_MAX_VEL, joint_interpolation_max_velocity);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, JOINT_INTERPOLATION_MAX_ACCEL, joint_interpolation_max_acceleration);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, JOINT_INTERPOLATION_MAX_JERK, joint_interpolation_max_jerk);
}

/**
 * Updates the JointTask after a controller cycle.
 * @param joint_task    The JointTask instance to update after a controller cycle
 */
void update_joint_task(Sai2Primitives::JointTask *joint_task)
{
    int dof = joint_task->_robot->dof();

    // re-initialize if new cycle enabled interpolation - otherwise interpolation has stale position internally
    if (joint_use_interpolation && !joint_task->_use_interpolation_flag)
        joint_task->reInitializeTask();

    if (!joint_task->_use_isotropic_gains && joint_use_isotropic_gains)
    {
        // going from nonisotropic to isotropic: take the median Kp & Kv
        VectorXd kp_median_temp(joint_kp_nonisotropic);
        VectorXd kv_median_temp(joint_kv_nonisotropic);

        std::nth_element(kp_median_temp.data(), kp_median_temp.data() + dof / 2, kp_median_temp.data() + dof);
        std::nth_element(kv_median_temp.data(), kv_median_temp.data() + dof / 2, kv_median_temp.data() + dof);

        joint_task->_kp = kp_median_temp[dof / 2];
        joint_task->_kv = kv_median_temp[dof / 2];
        redis_client.set(KP_JOINT_KEY, std::to_string(joint_task->_kp));
        redis_client.set(KV_JOINT_KEY, std::to_string(joint_task->_kv));
    }
    else if (joint_task->_use_isotropic_gains && !joint_use_isotropic_gains)
    {
        // going from isotropic to nonisotropic: set each joint to same value
        joint_kp_nonisotropic = joint_task->_kp * VectorXd::Ones(dof);
        joint_kv_nonisotropic = joint_task->_kv * VectorXd::Ones(dof);
        joint_task->setNonIsotropicGains(joint_kp_nonisotropic, joint_kv_nonisotropic, VectorXd::Zero(dof));
        redis_client.setEigenMatrixJSON(KP_NON_ISOTROPIC_JOINT_KEY, joint_kp_nonisotropic);
        redis_client.setEigenMatrixJSON(KV_NON_ISOTROPIC_JOINT_KEY, joint_kv_nonisotropic);
    }

    joint_task->setNonIsotropicGains(joint_kp_nonisotropic, joint_kv_nonisotropic, VectorXd::Zero(dof));
    joint_task->_use_interpolation_flag = bool(joint_use_interpolation);
    joint_task->_use_velocity_saturation_flag = bool(joint_use_velocity_saturation);
    joint_task->_use_isotropic_gains = bool(joint_use_isotropic_gains);

    joint_task->_otg->setMaxVelocity(joint_interpolation_max_velocity);
    joint_task->_otg->setMaxAcceleration(joint_interpolation_max_acceleration);
    joint_task->_otg->setMaxJerk(joint_interpolation_max_jerk);

    if (joint_dynamic_decoupling_mode == "full")
        joint_task->setDynamicDecouplingFull();
    else if (joint_dynamic_decoupling_mode == "bounded_inertia_estimate")
        joint_task->setDynamicDecouplingInertiaSaturation();
        // joint_task->setDynamicDecouplingBIE();
    else if (joint_dynamic_decoupling_mode == "none")
        joint_task->setDynamicDecouplingNone();
}

////////////////// POSORI TASK VARIABLES //////////////////
/** Flag to use PosOriTask with OTG interpolation or not */
int posori_use_interpolation;

/** PosOriTask OTG interpolation max linear velocity */
double posori_interpolation_max_linear_velocity;

/** PosOriTask OTG interpolation max linear acceleration */
double posori_interpolation_max_linear_acceleration;

/** PosOriTask OTG interpolation max linear jerk */
double posori_interpolation_max_linear_jerk;

/** PosOriTask OTG interpolation max angular velocity */
double posori_interpolation_max_angular_velocity;

/** PosOriTask OTG interpolation max angular acceleration */
double posori_interpolation_max_angular_acceleration;

/** PosOriTask OTG interpolation max angular jerk */
double posori_interpolation_max_angular_jerk;

/** Flag to use PosOri velocity saturation or not */
int posori_use_velocity_saturation;

/** Flag to use PosOri isotropic gains or not*/
int posori_use_isotropic_gains;

/** Tuple of (linear velocity, angular velocity) for velocity saturation */
Eigen::Vector2d posori_velocity_saturation;

/** Kp (x, y, z) when using nonisotropic gains */
Eigen::Vector3d posori_kp_nonisotropic;

/** Kv (x, y, z) when using nonisotropic gains */
Eigen::Vector3d posori_kv_nonisotropic;

/** Ki (x, y, z) when using nonisotropic gains */
Eigen::Vector3d posori_ki_nonisotropic;

/** Current dynamic decoupling mode: (full, partial, bounded_inertia_estimate, none)*/
std::string posori_dynamic_decoupling_mode;

/**
 * Initializes a PosOriTask object with default gains/settings and sets up Redis callbacks.
 * @param posori_task   The PosOriTask to initialize
 * @param redis_client  The RedisClient to use when setting up callbacks
 */
void init_posori_task(Sai2Primitives::PosOriTask *posori_task, RedisClient& redis_client)
{
    Matrix3d initial_orientation;
    Vector3d initial_position;
    Vector3d initial_velocity;

    int dof = posori_task->_robot->dof();
    posori_task->_robot->rotation(initial_orientation, posori_task->_link_name);
    posori_task->_robot->position(initial_position, posori_task->_link_name, posori_task->_control_frame.translation());
    posori_task->_robot->linearVelocity(initial_velocity, posori_task->_link_name, posori_task->_control_frame.translation());

    // initialize global variables
    // posori_use_interpolation = 0;
    posori_use_interpolation = true;
    posori_use_velocity_saturation = false;
    posori_use_isotropic_gains = true;
    posori_velocity_saturation = M_PI / 3.0 * Vector2d::Ones();
    // posori_kp_nonisotropic = 50.0 * Vector3d::Ones();
    // posori_kv_nonisotropic = 12.0 * Vector3d::Ones();
    posori_kp_nonisotropic = 100.0 * Vector3d::Ones();
    posori_kv_nonisotropic = 17.0 * Vector3d::Ones();
    posori_ki_nonisotropic = Vector3d::Zero();
    posori_dynamic_decoupling_mode = "full";
    posori_interpolation_max_linear_velocity = 0.3;
    posori_interpolation_max_linear_acceleration = 1.0;
    // posori_interpolation_max_linear_jerk = 3.0;
    posori_interpolation_max_linear_jerk = 5.0;
    // posori_interpolation_max_angular_velocity = M_PI / 3;
    // posori_interpolation_max_angular_acceleration = M_PI;
    // posori_interpolation_max_angular_jerk = 3 * M_PI;
    posori_interpolation_max_angular_velocity = M_PI / 1.5;
    posori_interpolation_max_angular_acceleration = 3.0 * M_PI;
    posori_interpolation_max_angular_jerk = 15.0 * M_PI;
    posori_velocity_saturation(0) = posori_task->_linear_saturation_velocity;
    posori_velocity_saturation(1) = posori_task->_angular_saturation_velocity;

    // initialize posori_task object
    posori_task->_use_interpolation_flag = bool(posori_use_interpolation);
    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    // posori_task->_kp_pos = 50.0;
    // posori_task->_kv_pos = 12.0;
    // posori_task->_ki_pos = 0.0;
    // posori_task->_kp_ori = 50.0;
    // posori_task->_kv_ori = 12.0;
    // posori_task->_ki_ori = 0.0;
    posori_task->_kp_pos = 100.0;
    posori_task->_kv_pos = 17.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 200.0;
    posori_task->_kv_ori = 23.0;
    posori_task->_ki_ori = 0.0;
    posori_task->setNonIsotropicGainsPosition(
        Matrix3d::Identity(),
        posori_kp_nonisotropic,
        posori_kv_nonisotropic,
        posori_ki_nonisotropic
    );
    posori_task->_use_isotropic_gains_position = bool(posori_use_isotropic_gains);
    posori_task->_use_isotropic_gains_orientation = true;
    posori_task->setDynamicDecouplingFull();

    posori_task->_otg->setMaxLinearVelocity(posori_interpolation_max_linear_velocity);
    posori_task->_otg->setMaxLinearAcceleration(posori_interpolation_max_linear_acceleration);
    posori_task->_otg->setMaxLinearJerk(posori_interpolation_max_linear_jerk);
    posori_task->_otg->setMaxAngularVelocity(posori_interpolation_max_angular_velocity);
    posori_task->_otg->setMaxAngularAcceleration(posori_interpolation_max_angular_acceleration);
    posori_task->_otg->setMaxAngularJerk(posori_interpolation_max_angular_jerk);

    // prepare redis callback
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, POSORI_USE_INTERPOLATION, posori_use_interpolation);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_VEL, posori_interpolation_max_linear_velocity);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_ACCEL, posori_interpolation_max_linear_acceleration);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_JERK, posori_interpolation_max_linear_jerk);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_VEL, posori_interpolation_max_angular_velocity);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL, posori_interpolation_max_angular_acceleration);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_JERK, posori_interpolation_max_angular_jerk);
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KI_POS_KEY, posori_task->_ki_pos);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KP_ORI_KEY, posori_task->_kp_ori);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KV_ORI_KEY, posori_task->_kv_ori);
    redis_client.addDoubleToReadCallback(READ_CALLBACK_ID, KI_ORI_KEY, posori_task->_ki_ori);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, KP_NONISOTROPIC_POS_KEY, posori_kp_nonisotropic);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, KV_NONISOTROPIC_POS_KEY, posori_kv_nonisotropic);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, KI_NONISOTROPIC_POS_KEY, posori_ki_nonisotropic);
    redis_client.addIntToReadCallback(READ_CALLBACK_ID, USE_ISOTROPIC_POS_GAINS_KEY, posori_use_isotropic_gains);
    redis_client.addStringToReadCallback(READ_CALLBACK_ID, DYN_DEC_POSORI_KEY, posori_dynamic_decoupling_mode);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_POS_KEY, posori_task->_desired_position);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_ORI_KEY, posori_task->_desired_orientation);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, DESIRED_VEL_KEY, posori_task->_desired_velocity);

    // update redis for initial conditions and any controller-induced changes
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_USE_INTERPOLATION, posori_use_interpolation);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_VEL, posori_interpolation_max_linear_velocity);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_ACCEL, posori_interpolation_max_linear_acceleration);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_LINEAR_JERK, posori_interpolation_max_linear_jerk);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_VEL, posori_interpolation_max_angular_velocity);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL, posori_interpolation_max_angular_acceleration);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, POSORI_INTERPOLATION_MAX_ANGULAR_JERK, posori_interpolation_max_angular_jerk);
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, USE_VEL_SAT_POSORI_KEY, posori_use_velocity_saturation);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, VEL_SAT_POSORI_KEY, posori_velocity_saturation);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_POS_KEY, posori_task->_kp_pos);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_POS_KEY, posori_task->_kv_pos);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KI_POS_KEY, posori_task->_ki_pos);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_ORI_KEY, posori_task->_kp_ori);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_ORI_KEY, posori_task->_kv_ori);
    redis_client.addDoubleToWriteCallback(INIT_WRITE_CALLBACK_ID, KI_ORI_KEY, posori_task->_ki_ori);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, KP_NONISOTROPIC_POS_KEY, posori_kp_nonisotropic);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, KV_NONISOTROPIC_POS_KEY, posori_kv_nonisotropic);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, KI_NONISOTROPIC_POS_KEY, posori_ki_nonisotropic);
    redis_client.addIntToWriteCallback(INIT_WRITE_CALLBACK_ID, USE_ISOTROPIC_POS_GAINS_KEY, posori_use_isotropic_gains);
    redis_client.addStringToWriteCallback(INIT_WRITE_CALLBACK_ID, DYN_DEC_POSORI_KEY, posori_dynamic_decoupling_mode);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_POS_KEY, posori_task->_desired_position);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_ORI_KEY, posori_task->_desired_orientation);
    redis_client.addEigenToWriteCallback(INIT_WRITE_CALLBACK_ID, DESIRED_VEL_KEY, posori_task->_desired_velocity);
}

/**
 * Updates a PosOriTask instance after a controller cycle.
 * @param posori_task   The PosOriTask instance to update after a controller cycle
 */
void update_posori_task(Sai2Primitives::PosOriTask *posori_task)
{
    int dof = posori_task->_robot->dof();

    if (posori_use_interpolation && !posori_task->_use_interpolation_flag)
        posori_task->reInitializeTask();

    if (posori_task->_use_isotropic_gains_position && !posori_use_isotropic_gains)
    {
        // going from isotropic to nonisotropic: set all Kp/Kv/Ki to same value
        posori_kp_nonisotropic = posori_task->_kp_pos * Vector3d::Ones();
        posori_kv_nonisotropic = posori_task->_kv_pos * Vector3d::Ones();
        posori_ki_nonisotropic = posori_task->_ki_pos * Vector3d::Ones();

        posori_task->setNonIsotropicGainsPosition(
            Matrix3d::Identity(),
            posori_kp_nonisotropic,
            posori_kv_nonisotropic,
            posori_ki_nonisotropic
        );
        redis_client.setEigenMatrixJSON(KP_NONISOTROPIC_POS_KEY, posori_kp_nonisotropic);
        redis_client.setEigenMatrixJSON(KV_NONISOTROPIC_POS_KEY, posori_kv_nonisotropic);
        redis_client.setEigenMatrixJSON(KI_NONISOTROPIC_POS_KEY, posori_ki_nonisotropic);
    }
    else if (!posori_task->_use_isotropic_gains_position && posori_use_isotropic_gains)
    {
        // going from nonisotropic to isotropic: set Kp/Kv to median
        Vector3d kp_median_temp(posori_kp_nonisotropic);
        Vector3d kv_median_temp(posori_kv_nonisotropic);

        std::nth_element(kp_median_temp.data(), kp_median_temp.data() + 1, kp_median_temp.data() + 3);
        std::nth_element(kv_median_temp.data(), kv_median_temp.data() + 1, kv_median_temp.data() + 3);

        posori_task->_kp_pos = kp_median_temp[1];
        posori_task->_kv_pos = kv_median_temp[1];
        redis_client.set(KP_POS_KEY, std::to_string(posori_task->_kp_pos));
        redis_client.set(KV_POS_KEY, std::to_string(posori_task->_kv_pos));
    }

    posori_task->_use_interpolation_flag = bool(posori_use_interpolation);
    posori_task->_otg->setMaxLinearVelocity(posori_interpolation_max_linear_velocity);
    posori_task->_otg->setMaxLinearAcceleration(posori_interpolation_max_linear_acceleration);
    posori_task->_otg->setMaxLinearJerk(posori_interpolation_max_linear_jerk);
    posori_task->_otg->setMaxAngularVelocity(posori_interpolation_max_angular_velocity);
    posori_task->_otg->setMaxAngularAcceleration(posori_interpolation_max_angular_acceleration);
    posori_task->_otg->setMaxAngularJerk(posori_interpolation_max_angular_jerk);

    posori_task->_use_velocity_saturation_flag = bool(posori_use_velocity_saturation);
    posori_task->_use_isotropic_gains_position = bool(posori_use_isotropic_gains);
    posori_task->_linear_saturation_velocity = posori_velocity_saturation(0);
    posori_task->_angular_saturation_velocity = posori_velocity_saturation(1);

    if (posori_dynamic_decoupling_mode == "full")
        posori_task->setDynamicDecouplingFull();
    else if (posori_dynamic_decoupling_mode == "partial")
        posori_task->setDynamicDecouplingPartial();
    else if (posori_dynamic_decoupling_mode == "bounded_inertia_estimate")
        posori_task->setDynamicDecouplingInertiaSaturation();
        // posori_task->setDynamicDecouplingBIE();
    else if (posori_dynamic_decoupling_mode == "none")
        posori_task->setDynamicDecouplingNone();
}

int main(int argc, char **argv)
{
    // open redis
    redis_client.connect();

    redis_client.createReadCallback(READ_CALLBACK_ID);
    redis_client.createWriteCallback(CONTROL_WRITE_CALLBACK_ID);
    redis_client.createWriteCallback(INIT_WRITE_CALLBACK_ID);
    redis_client.createWriteCallback(INIT_JOINT_TASK_WRITE_CALLBACK_ID);
    redis_client.createWriteCallback(INIT_POSORI_TASK_WRITE_CALLBACK_ID);

    // set up signal handlers
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // initialize controller state
    redis_client.set(PRIMITIVE_KEY, currentPrimitive);

    // notify UI that we are initializing
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZING);

    // load robots
    Affine3d T_world_robot = Affine3d::Identity();
    T_world_robot.translation() = Vector3d(0.0, 0.0, 0.0);
    auto robot = new Sai2Model::Sai2Model(ROBOT_FILE, false, T_world_robot);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(READ_CALLBACK_ID, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.executeReadCallback(READ_CALLBACK_ID);
    robot->updateModel();

    // bind current state to what redis says
    redis_client.addStringToReadCallback(READ_CALLBACK_ID, PRIMITIVE_KEY, currentPrimitive);

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);

    // control link and point for posori task
    const std::string link_name = "end_effector";
    const Vector3d pos_in_link = Vector3d(0.0,0.0,0.09);

    // sensor frame transform in end-effector frame
    Affine3d sensor_transform_in_link = Affine3d::Identity();
    const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);

    // read robot mass and coriolis from redis
    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);
    if(!flag_simulation) {
        redis_client.addEigenToReadCallback(READ_CALLBACK_ID, MASSMATRIX_KEY, mass_from_robot);
        redis_client.addEigenToReadCallback(READ_CALLBACK_ID, CORIOLIS_KEY, coriolis_from_robot);
    }

    // initialize tasks
    Sai2Primitives::PosOriTask *posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
    init_posori_task(posori_task, redis_client);

    Sai2Primitives::JointTask *joint_task = new Sai2Primitives::JointTask(robot);
    init_joint_task(joint_task, redis_client);

    Sai2Primitives::JointTask *floating_task = new Sai2Primitives::JointTask(robot);
    floating_task->_kp = 0;
    floating_task->_kv = FLOATING_TASK_KV;
    floating_task->_use_interpolation_flag = false;

    // prepare redis write callbacks for initializing each task
    redis_client.addEigenToWriteCallback(INIT_POSORI_TASK_WRITE_CALLBACK_ID, DESIRED_POS_KEY, posori_task->_current_position);
    redis_client.addEigenToWriteCallback(INIT_POSORI_TASK_WRITE_CALLBACK_ID, DESIRED_ORI_KEY, posori_task->_current_orientation);
    redis_client.addEigenToWriteCallback(INIT_JOINT_TASK_WRITE_CALLBACK_ID, DESIRED_JOINT_POS_KEY, robot->_q);

    // write robot end-effector position and velocity to redis
    Vector3d current_pos;
    Vector3d current_vel;
    robot->position(current_pos, link_name, pos_in_link);
    robot->linearVelocity(current_vel, link_name, pos_in_link);
    redis_client.addEigenToWriteCallback(CONTROL_WRITE_CALLBACK_ID, CURRENT_EE_POS_KEY, current_pos);
    redis_client.addEigenToWriteCallback(CONTROL_WRITE_CALLBACK_ID, CURRENT_EE_VEL_KEY, current_vel);

    // prepare redis write callback for controller loop
    redis_client.addEigenToWriteCallback(CONTROL_WRITE_CALLBACK_ID, JOINT_TORQUES_COMMANDED_KEY, command_torques);

    // initialization complete
    redis_client.executeWriteCallback(INIT_WRITE_CALLBACK_ID);
    redis_client.set(CONTROL_STATE_KEY, CONTROL_STATE_INITIALIZED);

    MatrixXd N_prec;

    // create a loop timer
    double control_freq = 1000;
    LoopTimer timer;
    timer.setLoopFrequency(control_freq);   // 1 KHz
    double last_time = timer.elapsedTime(); // secs
    bool fTimerDidSleep = true;
    timer.initializeTimer(1000000); // 1 ms pause before starting loop

    unsigned long long controller_counter = 0;

    runloop = true;
    while (runloop)
    {
        fTimerDidSleep = timer.waitForNextLoop();

        // update time
        double curr_time = timer.elapsedTime();
        double loop_dt = curr_time - last_time;

        std::string oldPrimitive = currentPrimitive;

        // update all values tied to redis
        redis_client.executeReadCallback(READ_CALLBACK_ID);
        update_joint_task(joint_task);
        update_posori_task(posori_task);

        if (flag_simulation)
        {
            robot->updateModel();
            robot->coriolisForce(coriolis);
        }
        else
        {
            robot->updateKinematics();
            // robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
            // robot->_M_inv = robot->_M.inverse();
            // coriolis = redis_client.getEigenMatrixJSON(CORIOLIS_KEY);
            robot->_M = mass_from_robot;
            robot->updateInverseInertia();
            coriolis = coriolis_from_robot;
        }

        N_prec.setIdentity(dof, dof);

        // if we just changed primitives, reset & reinit
        if (currentPrimitive != oldPrimitive)
        {
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                joint_task->_current_position = robot->_q;
                joint_task->reInitializeTask();
                // redis_client.setEigenMatrixJSON(DESIRED_JOINT_POS_KEY, robot->_q);
                redis_client.executeWriteCallback(INIT_JOINT_TASK_WRITE_CALLBACK_ID);
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                posori_task->reInitializeTask();
                // redis_client.setEigenMatrixJSON(DESIRED_POS_KEY, posori_task->_current_position);
                // redis_client.setEigenMatrixJSON(DESIRED_ORI_KEY, posori_task->_current_orientation);
                redis_client.executeWriteCallback(INIT_POSORI_TASK_WRITE_CALLBACK_ID);
            }
            else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
            {
                floating_task->_current_position = robot->_q;
                floating_task->reInitializeTask();
            }
        }

        // steady-state operations for each task
        else if (currentPrimitive == PRIMITIVE_JOINT_TASK)
        {
            joint_task->updateTaskModel(N_prec);
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }
        else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
        {
            joint_task->_use_isotropic_gains = true;
            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;
            joint_task->updateTaskModel(N_prec);

#ifdef USING_OTG
            // disable OTG for trajectory task
            if (currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
                redis_client.set(POSORI_USE_INTERPOLATION, "0");
#endif
            // we also need to read linear & angular velocity
            posori_task->_desired_angular_velocity.setZero();

            // compute torques
            posori_task->computeTorques(posori_task_torques);
            joint_task->computeTorques(joint_task_torques);
            command_torques = posori_task_torques + joint_task_torques + coriolis;
        }
        else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
        {
            Eigen::VectorXd floating_task_torques;
            floating_task->updateTaskModel(N_prec);
            floating_task->computeTorques(floating_task_torques);
            command_torques = floating_task_torques + coriolis;
        }

        // -------------------------------------------
        // redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

        // // log current EE position and velocity to redis
        // Vector3d current_pos;
        // robot->position(current_pos, link_name, pos_in_link);

        // Vector3d current_vel;
        // robot->linearVelocity(current_vel, link_name, pos_in_link);

        // redis_client.setEigenMatrixJSON(CURRENT_EE_POS_KEY, current_pos);
        // redis_client.setEigenMatrixJSON(CURRENT_EE_VEL_KEY, current_vel);

        // update robot end-effector position and velocity
        robot->position(current_pos, link_name, pos_in_link);
        robot->linearVelocity(current_vel, link_name, pos_in_link);

        // update redis values for current control cycle
        redis_client.executeWriteCallback(CONTROL_WRITE_CALLBACK_ID);

        // -------------------------------------------
        if (controller_counter % 500 == 0)
        {
            std::cout << "current primitive: " << currentPrimitive << std::endl;
            if (currentPrimitive == PRIMITIVE_JOINT_TASK)
            {
                std::cout << curr_time << std::endl;
                std::cout << "desired position : " << joint_task->_desired_position.transpose() << std::endl;
                std::cout << "current position : " << joint_task->_current_position.transpose() << std::endl;
                std::cout << "position error : " << (joint_task->_desired_position - joint_task->_current_position).norm() << std::endl;
                std::cout << std::endl;
            }
            else if (currentPrimitive == PRIMITIVE_POSORI_TASK || currentPrimitive == PRIMITIVE_TRAJECTORY_TASK)
            {
                std::cout << curr_time << std::endl;
                std::cout << "desired position : " << posori_task->_desired_position.transpose() << std::endl;
                std::cout << "current position : " << posori_task->_current_position.transpose() << std::endl;
                std::cout << "position error : " << (posori_task->_desired_position - posori_task->_current_position).norm() << std::endl;
                std::cout << std::endl;
            }
            else if (currentPrimitive == PRIMITIVE_FLOATING_TASK)
            {
                cout << curr_time << endl;
                cout << "current velocity : " << floating_task->_current_velocity.transpose() << endl;
                cout << "current velocity norm: " << floating_task->_current_velocity.norm() << endl;
                cout << endl;
            }
        }

        controller_counter++;

        // -------------------------------------------
        // update last time
        last_time = curr_time;
    }

    command_torques.setZero();
    redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << std::endl;
    std::cout << "Control Loop run time  : " << end_time << " seconds" << std::endl;
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << std::endl;
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz" << std::endl;

    delete robot;
    delete joint_task;
    delete posori_task;
    delete floating_task;
    return 0;
}
