
#include "p4_ros/services.h"

namespace p4_ros {

bool minTimeService(p4_ros::min_time::Request  &req,
                    p4_ros::min_time::Response &res) {
	ROS_INFO("[p4_ros] Running minimum time solver!");

	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_min_time_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	// ros::Time t0 = ros::Time::now();
	// const p4::PolynomialPath path =
	//     solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Find path through gradient descent optimizing segment times
	ros::Time t1 = ros::Time::now();
	p4::PolynomialSolver solver(solver_options);
	p4::PolynomialPath path_optimized;
	std::vector<double> times_final;
	p4_helper::solve_optimal_time_problem(times, node_eq, segment_ineq,
		                                  solver_options, node_ineq,
		                                  &times_final, &path_optimized);
	ros::Time t2 = ros::Time::now();
	// ROS_WARN("First solver time: %f", (t1-t0).toSec());
	ROS_WARN("Trajectory generation time: %f", (t2-t1).toSec());

	// Convert segments from Tucker-polynomials (used in P4) into the form expected by the minimum time solver
	// Dfining dt = t_seg_final - t_seg_initial, Tucker polynomials are written as follows:
	// p(t) = [a0 a1 a2 ... a3]*[1 t/dt t^2/(2! dt^2) t^3/(3! dt^3) ... t^n/(n! dt^n)]
	const Eigen::VectorXd segment_times = p4_helper::time_to_segment_time(times_final);
	const uint n_seg = segment_times.size();
	const uint n_coeff = path_optimized.coefficients[0].col(0).size();
	Eigen::VectorXd segment_X, segment_Y, segment_Z;
	Eigen::MatrixXd coeff_matrix(n_seg, 3*n_coeff);
	for (uint i = 0; i < n_seg; i++) {
		segment_X = path_optimized.coefficients[p4_helper::DimensionIdx::X].col(i);
		segment_Y = path_optimized.coefficients[p4_helper::DimensionIdx::Y].col(i);
		segment_Z = path_optimized.coefficients[p4_helper::DimensionIdx::Z].col(i);
		// std::cout << segment_X.transpose() << std::endl;

		// Divide each term by the factorial, because the polynomials in P4 are scaled that way
		for (uint j = 0; j < segment_X.size(); j++) {
			double factorial = p4_helper::factorial(j);
			double dt_factor = pow(segment_times[i], j);
			segment_X[j] = segment_X[j]/(factorial*dt_factor);
			segment_Y[j] = segment_Y[j]/(factorial*dt_factor);
			segment_Z[j] = segment_Z[j]/(factorial*dt_factor);
		}

		// Concatenate the segments into a single vector
		Eigen::VectorXd concatenation(3*segment_X.size());
		concatenation << segment_X, segment_Y, segment_Z;
		coeff_matrix.row(i) = concatenation;
	}

	// Run the time optimizer
	double d_s = 0.02, rho = 0.0, tf;
	uint poly_order = n_coeff - 1;

	TimeOptimizerClass time_optimizer_obj(req.max_vel, req.max_acc, req.max_jerk,
        d_s, rho, poly_order, req.sampling_freq, coeff_matrix, segment_times,
        req.visualize_output, &res.pva_vec, &res.final_time);

	// p4_helper::plot_results_3d(times, path);
	// p4_helper::plot_results_3d(times_final, path_optimized);
	// p4_helper::plot_results(times_final, path_optimized);

	// std::cout << "Initial times: " << std::endl;
	// for (uint i = 0; i < times.size(); i++) {
	// 	std::cout << times[i] << "\t";
	// }
	// std::cout << std::endl;

	// std::cout << "Final times: " << std::endl;
	// for (uint i = 0; i < times_final.size(); i++) {
	// 	std::cout << times_final[i] << "\t";
	// }
	// std::cout << std::endl;

	return true;
}

bool minAccXYService(p4_ros::minAccXYWpPVA::Request  &req,
                     p4_ros::minAccXYWpPVA::Response &res) {
	ROS_INFO("[p4_ros] Running minimum acceleration XY solver!");

	// Problem variables
	std::vector<double> times;
	std::vector<p4::NodeEqualityBound> node_eq;
	std::vector<p4::SegmentInequalityBound> segment_ineq;
	p4::PolynomialSolver::Options solver_options;
	std::vector<p4::NodeInequalityBound> node_ineq;  // We leave this structure empty in the current service

	// Setup optimization problem
	p4_helper::setup_min_acc_problem(req, &times, &node_eq, &segment_ineq, &solver_options);

	// Solve problem
	p4::PolynomialSolver solver(solver_options);
	const p4::PolynomialPath path =
	    solver.Run(times, node_eq, node_ineq, segment_ineq);

	// Return structure
	const uint n_seg = times.size() - 1;
	for (uint idx = 0; idx < n_seg; idx++) {
		p4_ros::PolyPVA X, Y;
		X = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::X);
		Y = p4_helper::segment_pva_coeff_from_path(path, times, idx, p4_helper::DimensionIdx::Y);
		res.X.push_back(X);
		res.Y.push_back(Y);
	}
	// res.cost = path.optimal_cost;

	// p4_helper::plot_results(times, path);

	ROS_INFO("[p4_services] Returning request for %zu waypoints.", times.size());

	return true;
}

}  // namespace p4_ros