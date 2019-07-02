
// ROS
#include "ros/ros.h"

// services
#include "p4_ros/services.h"

// Libraries to remove afterwards (debug/testing)
#include "make_sparse.h"
#include "time_optimizer_ecos.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p4_services");
	ros::NodeHandle node("~");

	ROS_INFO("[p4_services] Starting services!");
	p4_ros::ServicesClass services(&node);

	// vector<vector<int> > M = { 
 //        { 10, 20,  0,  0,  0,  0 }, 
 //        { 0,  30,  0, 40,  0,  0 }, 
 //        { 0,   0, 50, 60, 70,  0 }, 
 //        { 0,   0,  0,  0,  0, 80 }, 
 //    };
	// vector<vector<int> > M = { 
 //        { 10, 0, 0, 0, -2, 0 }, 
 //        { 3, 9, 0, 0, 0, 3 }, 
 //        { 0, 7, 8, 7, 0, 0 }, 
 //        { 3, 0, 8, 7, 5, 0 }, 
 //        { 0, 8, 0, 9, 9, 13 }, 
 //        { 0, 4, 0, 0, 2, -1 },
 //    };
	// Eigen::MatrixXd Me(4,4);
	// std::vector<double> A;
	// std::vector<int> IA, JA;
	// Me  << 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1;
	// make_sparse_ccs(Me, &A, &IA, &JA);
	// vector<vector<int> > M = { 
 //        { 1, 0, 0, 1 }, 
 //        { 1, 1, 0, 0 }, 
 //        { 0, 0, 1, 0 }, 
 //        { 1, 1, 0, 1 },
 //    };
 //    make_sparse_ccs(M); 

	// const uint m = 2;
	// std::vector<uint> K = {2, 3, 2};
	// ecos_sol::variable_set_xyz obj(m, K);
	// std::cout << obj.final_index_ << std::endl;
	// std::cout << obj.x_.a_.initial_index_ << " " << obj.x_.a_.final_index_ << std::endl;
	// std::cout << obj.x_.b_.initial_index_ << " " << obj.x_.b_.final_index_ << std::endl;
	// std::cout << obj.x_.c_.initial_index_ << " " << obj.x_.c_.final_index_ << std::endl;
	// std::cout << obj.x_.d_.initial_index_ << " " << obj.x_.d_.final_index_ << std::endl;

	// std::cout << std::endl;
	// for (uint i = 0; i <= m; i++) {
	// 	for (uint j = 0; j < K[i]; j++) {
	// 		// std::cout << i << " " << j << " ";
	// 		std::cout << obj.get_index(ecos_sol::var_direction::z, ecos_sol::var_names::a, i, j) << " ";
	// 	}
	// 	// std::cout << std::endl;
	// }

	// std::cout << std::endl;
	// for (uint i = 0; i <= m; i++) {
	// 	for (uint j = 0; j <= K[i]; j++) {
	// 		std::cout << obj.get_index(ecos_sol::var_direction::z, ecos_sol::var_names::b, i, j) << " ";
	// 	}
	// }

	// std::cout << std::endl;
	// for (uint i = 0; i <= m; i++) {
	// 	for (uint j = 0; j <= K[i]; j++) {
	// 		std::cout << obj.get_index(ecos_sol::var_direction::z, ecos_sol::var_names::c, i, j) << " ";
	// 	}
	// }

	// std::cout << std::endl;
	// for (uint i = 0; i <= m; i++) {
	// 	for (uint j = 0; j < K[i]; j++) {
	// 		std::cout << obj.get_index(ecos_sol::var_direction::z, ecos_sol::var_names::d, i, j) << " ";
	// 	}
	// }
	// std::cout << std::endl;

	ros::spin();

    return 0;
}