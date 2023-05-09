#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d q_dot_desired_; // Control value
	Vector7d torque_desired_;

	// Task space
	Vector3d x_init_;
	Vector3d x_2_init_;
	Vector3d x_;
	Vector3d x_desired_;
	Vector3d x_dot_desired_;
	Matrix3d rotation_;
	Matrix3d rotation_init_;
	Matrix3d rotation_desired_;
	Vector3d phi_;
	Vector6d x_dot_; // 6D (linear + angular)
	Vector6d x_error_;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix

	// For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d q_target_;

	Vector3d x_cubic_;
	Vector6d xd_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;
	Matrix3d rotation_target_;
	Matrix3d rotation_cubic_;

	Vector3d x_2_; //4번 링크 CoM 위치
	Matrix<double, 6, 7> j_2_; //4번 링크 CoM의 jacobian matrix
	MatrixXd j_temp_2_; //각각 x_, j_, j_temp_2 변수 선언 밑에 작성
	Matrix<double, 3, 7> j_v_2_; 

	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;

	std::string control_mode_;
	bool is_mode_changed_;
	bool is_mode_init_ = true;

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];

	// Kwan add
	Vector7d qdot_desired_;
	Eigen::Vector7d q_diff_;

private:
	void printState();
	void moveJointPosition(const Vector7d &target_position, double duration);
	void moveJointPositionTorque(const Vector7d &target_position, double duration);
	void simpleJacobianControl(const Vector12d & target_x, double duration);
	void feedbackJacobianControl(const Vector12d & target_x, double duration);
	void CLIK(const Vector12d & target_x, double duration);

	Eigen::Isometry3d PositionUpdate(Eigen::Vector7d qd_, int Joint_idx);
	Eigen::MatrixXd JacobianUpdate(Eigen::Vector7d qd_, int Joint_idx);
	
public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initModel(); initFile();
	}


    void setMode(const std::string & mode);
    void initDimension();
    void initModel();
	void initFile();
    void initPosition();
    void compute();
private:
	ofstream debug_file_;
	constexpr static int NUM_HW_PLOT{15};
	ofstream hw_plot_files_[NUM_HW_PLOT];
	const string hw_plot_file_names_[NUM_HW_PLOT]
	{"/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-1",
	 "/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-2",
	 "/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-3",
	 "/home/kwan/kwan_robotics_assignment/graph/HW3/HW3-4"};

	void record(int file_number, double duration);
	void record(int file_number, double duration, const stringstream & ss);
	void recordHw3(int file_number, double duration, const Vector3d & traj1, const Vector3d & traj2);
	void recordHw3(int file_number, double duration, const Vector3d & traj2, double h1, double h2);
	void recordHw4(int file_number, double duration, const Vector7d & traj = Vector7d::Zero());
};
