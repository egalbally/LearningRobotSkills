#ifndef _KEYS_H
#define _KEYS_H

// simulation keys
constexpr const char *SIM_JOINT_ANGLES_KEY = "sai2::HapticApplications::04::simviz::sensors::q";
constexpr const char *SIM_JOINT_VELOCITIES_KEY = "sai2::HapticApplications::04::simviz::sensors::dq";
constexpr const char *SIM_JOINT_TORQUES_COMMANDED_KEY = "sai2::HapticApplications::04::simviz::actuators::fgc";

// used when on real hardware
constexpr const char *HW_JOINT_ANGLES_KEY = "sai2::FrankaPanda::sensors::q";
constexpr const char *HW_JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
constexpr const char *HW_JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
constexpr const char *MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
constexpr const char *ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";
constexpr const char *CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";

// useful logging variables
constexpr const char *CURRENT_EE_POS_KEY = "sai2::HapticApplications::04::current_ee_pos";
constexpr const char *CURRENT_EE_VEL_KEY = "sai2::HapticApplications::04::current_ee_vel";

// controller initialization
constexpr const char *CONTROL_STATE_KEY = "sai2::HapticApplications::04::control_state";
constexpr const char *CONTROL_STATE_INITIALIZING = "initializing";
constexpr const char *CONTROL_STATE_INITIALIZED = "initialized";
constexpr const char *CONTROL_STATE_READY = "ready";

// posori task parameters
constexpr const char *DESIRED_POS_KEY = "sai2::HapticApplications::04::desired_position";
constexpr const char *DESIRED_ORI_KEY = "sai2::HapticApplications::04::desired_orientation";
constexpr const char *DESIRED_VEL_KEY = "sai2::HapticApplications::04::desired_velocity";
constexpr const char *KP_POS_KEY = "sai2::HapticApplications::04::kp_pos";
constexpr const char *KV_POS_KEY = "sai2::HapticApplications::04::kv_pos";
constexpr const char *KI_POS_KEY = "sai2::HapticApplications::04::ki_pos";
constexpr const char *KP_ORI_KEY = "sai2::HapticApplications::04::kp_ori";
constexpr const char *KV_ORI_KEY = "sai2::HapticApplications::04::kv_ori";
constexpr const char *KI_ORI_KEY = "sai2::HapticApplications::04::ki_ori";
constexpr const char *KP_NONISOTROPIC_POS_KEY = "sai2::HapticApplications::04::kp_nonisotropic_pos";
constexpr const char *KV_NONISOTROPIC_POS_KEY = "sai2::HapticApplications::04::kv_nonisotropic_pos";
constexpr const char *KI_NONISOTROPIC_POS_KEY = "sai2::HapticApplications::04::ki_nonisotropic_pos";
constexpr const char *USE_ISOTROPIC_POS_GAINS_KEY = "sai2::HapticApplications::04::use_isotropic_pos_gains";
constexpr const char *POSORI_USE_INTERPOLATION = "sai2::HapticApplications::04::posori_use_interpolation";
constexpr const char *POSORI_INTERPOLATION_MAX_LINEAR_VEL = "sai2::HapticApplications::04::posori_interpolation_max_linear_vel";
constexpr const char *POSORI_INTERPOLATION_MAX_LINEAR_ACCEL = "sai2::HapticApplications::04::posori_interpolation_max_linear_accel";
constexpr const char *POSORI_INTERPOLATION_MAX_LINEAR_JERK = "sai2::HapticApplications::04::posori_interpolation_max_linear_jerk";
constexpr const char *POSORI_INTERPOLATION_MAX_ANGULAR_VEL = "sai2::HapticApplications::04::posori_interpolation_max_angular_vel";
constexpr const char *POSORI_INTERPOLATION_MAX_ANGULAR_ACCEL = "sai2::HapticApplications::04::posori_interpolation_max_angular_accel";
constexpr const char *POSORI_INTERPOLATION_MAX_ANGULAR_JERK = "sai2::HapticApplications::04::posori_interpolation_max_angular_jerk";
constexpr const char *USE_VEL_SAT_POSORI_KEY = "sai2::HapticApplications::04::use_posori_velocity_saturation";
constexpr const char *VEL_SAT_POSORI_KEY = "sai2::HapticApplications::04::posori_velocity_saturation";
constexpr const char *DYN_DEC_POSORI_KEY = "sai2::HapticApplications::04::posori_dynamic_decoupling";

// joint task parameters
constexpr const char *DESIRED_JOINT_POS_KEY = "sai2::HapticApplications::04::desired_joint_position";
constexpr const char *KP_JOINT_KEY = "sai2::HapticApplications::04::kp_joint";
constexpr const char *KV_JOINT_KEY = "sai2::HapticApplications::04::kv_joint";
constexpr const char *KP_NON_ISOTROPIC_JOINT_KEY = "sai2::HapticApplications::04::kp_nonisotropic_joint";
constexpr const char *KV_NON_ISOTROPIC_JOINT_KEY = "sai2::HapticApplications::04::kv_nonisotropic_joint";
constexpr const char *USE_ISOTROPIC_JOINT_GAINS_KEY = "sai2::HapticApplications::04::use_isotropic_joint_gains";
constexpr const char *JOINT_USE_INTERPOLATION = "sai2::HapticApplications::04::joint_use_interpolation";
constexpr const char *JOINT_INTERPOLATION_MAX_VEL = "sai2::HapticApplications::04::joint_interpolation_max_vel";
constexpr const char *JOINT_INTERPOLATION_MAX_ACCEL = "sai2::HapticApplications::04::joint_interpolation_max_accel";
constexpr const char *JOINT_INTERPOLATION_MAX_JERK = "sai2::HapticApplications::04::joint_interpolation_max_jerk";
constexpr const char *USE_VEL_SAT_JOINT_KEY = "sai2::HapticApplications::04::use_joint_velocity_saturation";
constexpr const char *VEL_SAT_JOINT_KEY = "sai2::HapticApplications::04::joint_velocity_saturation";
constexpr const char *DYN_DEC_JOINT_KEY = "sai2::HapticApplications::04::joint_dynamic_decoupling";

// robot file
constexpr const char *ROBOT_FILE = "resources/panda_arm.urdf";

// state-related keys
constexpr const char *PRIMITIVE_KEY = "sai2::HapticApplications::04::primitive";
constexpr const char *PRIMITIVE_JOINT_TASK = "primitive_joint_task";
constexpr const char *PRIMITIVE_POSORI_TASK = "primitive_posori_task";
constexpr const char *PRIMITIVE_TRAJECTORY_TASK = "primitive_trajectory_task";
constexpr const char *PRIMITIVE_FLOATING_TASK = "primitive_floating_task";

#endif
