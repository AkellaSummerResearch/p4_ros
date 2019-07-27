
#ifndef P4_HELPER_H_
#define P4_HELPER_H_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "p4_ros/minAccXYWpPVA.h"
#include "p4_ros/min_time.h"
#include "p4_ros/trapezoidal_p2p.h"
#include "p4_ros/trapezoidal.h"
#include "p4_ros/geometry_functions.h"

#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"

namespace p4_helper {

namespace DimensionIdx {
static constexpr int X = 0;
static constexpr int Y = 1;
static constexpr int Z = 2;
}

namespace DerivativeIdx {
static constexpr int Position = 0;
static constexpr int Velocity = 1;
static constexpr int Acceleration = 2;
static constexpr int Jerk = 3;
static constexpr int Snap = 4;
static constexpr int Crackle = 5;
static constexpr int Pop = 6;
}

size_t factorial(size_t n);

double saturate(const double &value, const double &min_val,
	            const double &max_val);

std::vector<float> eigen_to_stdvector(const Eigen::VectorXd &eig_vec);

Eigen::VectorXd stdvector_to_eigen(const std::vector<double> &vec);

std::vector<double> scale_std_vector(const std::vector<double> &vec, 
	                                 const double factor);

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d &eig_vec);

geometry_msgs::Point ros_point(const double &x, const double &y, const double &z);

geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d &eig_vec);

geometry_msgs::Vector3 ros_vector3(const double &x, const double &y, const double &z);

p4_ros::PVA_4d constuct_PVA (const Eigen::Vector3d &pos,
	                         const Eigen::Vector3d &vel,
	                         const Eigen::Vector3d &acc,
	                         const double &yaw, const double &yaw_vel,
	                         const double &yaw_acc, const double &time);

std::vector<float> diff_coeff(const std::vector<float> &coeff_in);

void get_perpendicular_vectors(const Eigen::Vector3d &v, Eigen::Vector3d *v1, Eigen::Vector3d *v2);

void set_pos_eq_constraint(const geometry_msgs::Point &pos, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_vel_eq_constraint(const geometry_msgs::Vector3 &vel, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_acc_eq_constraint(const geometry_msgs::Vector3 &acc, const uint &node_idx,
	                       std::vector<p4::NodeEqualityBound> *eq_constraints);

void set_max_vel(const double &max_vel, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void set_max_acc(const double &max_acc, const uint &seg_idx,
	             std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void set_max_jerk(const double &max_jerk, const uint &seg_idx,
	              std::vector<p4::SegmentInequalityBound> *ineq_constraints);

void setup_min_time_problem(const p4_ros::min_time::Request &req,
							const bool &is_straight,
			                std::vector<double> *times,
						    std::vector<p4::NodeEqualityBound> *node_eq,
						    std::vector<p4::SegmentInequalityBound> *segment_ineq,
						    p4::PolynomialSolver::Options *solver_options);

void setup_min_acc_problem(const p4_ros::minAccXYWpPVA::Request &req,
			               std::vector<double> *times,
						   std::vector<p4::NodeEqualityBound> *node_eq,
						   std::vector<p4::SegmentInequalityBound> *segment_ineq,
						   p4::PolynomialSolver::Options *solver_options);

// Returns true if all waypoints are closely aligned
bool is_trajectory_straight(const std::vector<geometry_msgs::Point> &pts);

Eigen::VectorXd time_to_segment_time(const std::vector<double> &times);

std::vector<double> segment_time_to_time(const Eigen::VectorXd &segment_times);

void solve_initial_min_time_trajectory(const std::vector<double> &init_time_guess,
	                            	   const std::vector<p4::NodeEqualityBound> &node_eq,
									   const std::vector<p4::SegmentInequalityBound> &segment_ineq,
									   const p4::PolynomialSolver::Options &solver_options,
									   const std::vector<p4::NodeInequalityBound> &node_ineq,
									   std::vector<double> *times,
									   p4::PolynomialPath *path);

p4_ros::PolyPVA segment_pva_coeff_from_path(const p4::PolynomialPath &path,
	                                        const std::vector<double> &times,
	                                        const uint &seg_idx, const uint &dimension_idx);
void tucker_polynomials_to_coeff_matrix(
			const p4::PolynomialPath &path,
			const std::vector<double> &times,
			Eigen::VectorXd *segment_times, Eigen::MatrixXd *coeff_matrix);

void plot_results(const std::vector<double> &times, const p4::PolynomialPath &path);

void plot_results_3d(const std::vector<double> &times, const p4::PolynomialPath &path);

}  // namespace p4_helper

#endif  // P4_HELPER_H_