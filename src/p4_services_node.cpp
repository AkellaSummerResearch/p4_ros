
// ROS
#include "ros/ros.h"

// services
#include "p4_ros/services.h"

// Libraries to remove afterwards (debug/testing)
#include "p4_ros/trapezoidal.h"
#include "gnuplot-iostream.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p4_services");
	ros::NodeHandle node("~");

	// const Eigen::Vector3d init_pos(1.0, 0.0, 0.0);
	// const Eigen::Vector3d final_pos(3.0, 0.0, 0.0);
	// const double max_vel = 1.5;
	// const double max_acc = 0.3;
	// trapezoidal::planner tp(init_pos, final_pos, max_vel, max_acc);

	// const double sample_freq = 100;
	// std::vector<double> time_hist;
	// std::vector<Eigen::Vector3d> pos_hist, vel_hist, acc_hist;
	// tp.sample_trajectory(sample_freq, &time_hist, &pos_hist, &vel_hist, &acc_hist);	
	// tp.plot_traj_1D(time_hist, pos_hist, vel_hist, acc_hist);

	ROS_INFO("[p4_services] Starting services!");
	p4_ros::ServicesClass services(&node);

	ros::spin();

    return 0;
}